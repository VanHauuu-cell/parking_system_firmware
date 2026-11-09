#include "state_machine.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "fsm_config.h"
#include "parking_manager.h"
#include "access_manager.h"
#include "barrier_manager.h"
#include "display_manager.h"
#include "uart_manager.h"
#include "gate_ir_manager.h"
#include "system_config.h"

static const char *TAG = "PARKING_FSM";

static uint8_t camera_for_direction(parking_direction_t direction)
{
    return direction == PARKING_ENTRY ? FSM_ENTRY_CAMERA_ID : FSM_EXIT_CAMERA_ID;
}

const char *state_machine_name(app_state_t state)
{
    static const char *names[] = {
        "INIT", "IDLE", "WAIT_OCR", "OPENING", "WAIT_PASSAGE", "CLOSE_DELAY",
        "CLOSING", "NOTICE", "FAULT"
    };
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}

static void change_state(state_machine_t *fsm, app_state_t state, int64_t now_us)
{
    ESP_LOGI(TAG, "%s -> %s", state_machine_name(fsm->state), state_machine_name(state));
    fsm->state = state;
    fsm->state_since_us = now_us;
    fsm->deadline_us = 0;
}

static void display_idle(const state_machine_t *fsm)
{
    if (FSM_AUTO_DIRECTION) {
        char line[17];
        unsigned free_count = parking_manager_free_count();
        if (free_count > PARKING_SLOT_COUNT) { free_count = PARKING_SLOT_COUNT; }
        snprintf(line, sizeof(line), "FREE:%u/%u TAP", free_count, (unsigned)PARKING_SLOT_COUNT);
        display_manager_show("READY AUTO", line);
    } else { display_manager_idle(fsm->direction, parking_manager_free_count()); }
}

static void idle(state_machine_t *fsm, int64_t now_us)
{
    // Never lose an authorized transaction by merely changing the RAM state.
    if (parking_manager_get_pending(NULL)) {
        state_machine_fault(fsm, "PENDING EXISTS");
        return;
    }
    fsm->uid[0] = '\0';
    fsm->cancelled = false;
    fsm->passage_warning = false;
    change_state(fsm, APP_STATE_IDLE, now_us);
    display_idle(fsm);
    ESP_LOGI(TAG, "[READY] mode=%s; remove card, then tap",
             FSM_AUTO_DIRECTION ? "AUTO" : fsm->direction == PARKING_ENTRY ? "ENTRY" : "EXIT");
}

static void notice(state_machine_t *fsm, const char *reason, int64_t now_us)
{
    if (parking_manager_get_pending(NULL)) {
        state_machine_fault(fsm, reason);
        return;
    }
    change_state(fsm, APP_STATE_NOTICE, now_us);
    fsm->deadline_us = now_us + (int64_t)FSM_NOTICE_MS * 1000;
    ESP_LOGW(TAG, "[DENIED] uid=%s reason=%s", fsm->uid, reason);
    display_manager_show(reason, "Remove / retap");
}

void state_machine_fault(state_machine_t *fsm, const char *reason)
{
    change_state(fsm, APP_STATE_FAULT, esp_timer_get_time());
    barrier_manager_silence();
    // Keep PWM position, UID and durable pending record. Never force a close.
    display_manager_show("FAULT / CHECK", reason);
    ESP_LOGE(TAG, "[FAULT] %s; pending is preserved; no automatic gate movement", reason);
}

static void start_closing(state_machine_t *fsm, bool cancelled, bool manual)
{
    int64_t now_us = esp_timer_get_time();
    if (!gate_ir_manager_clear(fsm->direction, now_us)) {
        ESP_LOGW(TAG, "[HOLD] Gate IR must be stably CLEAR before closing");
        return;
    }
    fsm->cancelled = cancelled;
    fsm->manual_closing = manual;
    if (barrier_manager_close(fsm->direction, now_us) != ESP_OK) {
        state_machine_fault(fsm, "CLOSE ERROR");
        return;
    }
    change_state(fsm, APP_STATE_CLOSING, now_us);
    display_manager_show(cancelled ? "CANCEL / CLOSING" : "CLOSING", fsm->uid);
}

static void wait_passage(state_machine_t *fsm, int64_t now_us)
{
    change_state(fsm, APP_STATE_WAIT_PASSAGE, now_us);
    fsm->deadline_us = now_us + (int64_t)FSM_PASSAGE_WARNING_MS * 1000;
    fsm->passage_warning = false;
    display_manager_show("WAIT CAR PASS", "IR after barrier");
}

static void reopen(state_machine_t *fsm, int64_t now_us)
{
    ESP_LOGW(TAG, "[REOPEN] Gate IR blocked or unsettled");
    if (barrier_manager_open(fsm->direction, now_us) != ESP_OK) {
        state_machine_fault(fsm, "REOPEN ERROR");
        return;
    }
    if (fsm->manual_closing) {
        // The operator must reassess a cancelled/recovered passage after an obstruction.
        state_machine_fault(fsm, "RECHECK PASSAGE");
        return;
    }
    change_state(fsm, APP_STATE_OPENING, now_us);
    display_manager_show("REOPEN / HOLD", "Gate IR blocked");
}

static void new_card(state_machine_t *fsm, const char *uid, int64_t now_us)
{
    if (fsm->state != APP_STATE_IDLE) {
        ESP_LOGW(TAG, "[BUSY] ignore uid=%s while %s", uid, state_machine_name(fsm->state));
        return;
    }
    snprintf(fsm->uid, sizeof(fsm->uid), "%s", uid);
    const vehicle_registration_t *vehicle = parking_manager_find(uid);
    if (vehicle == NULL) { notice(fsm, "UNKNOWN CARD", now_us); return; }
    if (FSM_AUTO_DIRECTION) {
        fsm->direction = parking_manager_is_inside(uid) ? PARKING_EXIT : PARKING_ENTRY;
    }
    fsm->camera_id = camera_for_direction(fsm->direction);
    fsm->gate_seen_active = false;
    fsm->manual_closing = false;
    if (fsm->direction == PARKING_ENTRY && parking_manager_is_inside(uid)) {
        notice(fsm, "ALREADY INSIDE", now_us); return;
    }
    if (fsm->direction == PARKING_EXIT && !parking_manager_is_inside(uid)) {
        notice(fsm, "NOT INSIDE", now_us); return;
    }
    if (!parking_manager_storage_ready()) {
        state_machine_fault(fsm, "STORAGE ERROR"); return;
    }
    if (fsm->direction == PARKING_ENTRY && !parking_manager_slots_ready()) {
        notice(fsm, "SENSORS WAIT", now_us); return;
    }
    if (fsm->direction == PARKING_ENTRY && parking_manager_free_count() == 0) {
        notice(fsm, "PARKING FULL", now_us); return;
    }
    uint32_t id;
    do { id = esp_random(); } while (id == 0 || id == fsm->request_id);
    fsm->request_id = id;
    if (uart_manager_request(fsm->camera_id, id) != ESP_OK) {
        notice(fsm, "UART SEND ERROR", now_us); return;
    }
    change_state(fsm, APP_STATE_WAIT_OCR, now_us);
    fsm->deadline_us = now_us + (int64_t)FSM_OCR_TIMEOUT_MS * 1000;
    display_manager_show("READING PLATE", fsm->uid);
    ESP_LOGI(TAG, "[OCR REQUEST] uid=%s camera=%u request=%lu expected=%s",
             uid, (unsigned)fsm->camera_id, (unsigned long)id, vehicle->plate);
}

static void ocr_result(state_machine_t *fsm, const uart_ocr_result_t *result,
                       int64_t now_us)
{
    if (fsm->state != APP_STATE_WAIT_OCR || result->camera_id != fsm->camera_id ||
        result->request_id != fsm->request_id) {
        ESP_LOGW(TAG, "[IGNORED OCR] camera=%u request=%lu state=%s",
                 (unsigned)result->camera_id, (unsigned long)result->request_id,
                 state_machine_name(fsm->state));
        return;
    }
    if (now_us >= fsm->deadline_us) { notice(fsm, "OCR TIMEOUT", now_us); return; }
    if (!result->success) { notice(fsm, result->reason, now_us); return; }
    if (result->confidence < FSM_MIN_OCR_CONFIDENCE) {
        notice(fsm, "LOW CONFIDENCE", now_us); return;
    }
    access_decision_t decision = access_manager_check(fsm->uid, result->plate, fsm->direction);
    if (decision != ACCESS_ALLOWED) {
        notice(fsm, access_manager_reason(decision), now_us); return;
    }
    if (!gate_ir_manager_clear(fsm->direction, esp_timer_get_time())) {
        notice(fsm, "GATE IR BUSY", esp_timer_get_time());
        return;
    }
    // Grant is durable before the physical barrier can start moving.
    if (parking_manager_begin(fsm->uid, fsm->direction, fsm->request_id) != ESP_OK) {
        state_machine_fault(fsm, "SAVE GRANT ERROR"); return;
    }
    fsm->gate_seen_active = false;
    now_us = esp_timer_get_time();
    if (barrier_manager_open(fsm->direction, now_us) != ESP_OK) {
        state_machine_fault(fsm, "OPEN ERROR"); return;
    }
    change_state(fsm, APP_STATE_OPENING, now_us);
    display_manager_show(fsm->direction == PARKING_ENTRY ? "ENTRY GRANTED" : "EXIT GRANTED",
                         result->plate);
    ESP_LOGI(TAG, "[GRANTED] uid=%s plate=%s confidence=%u request=%lu", fsm->uid,
             result->plate, (unsigned)result->confidence, (unsigned long)fsm->request_id);
}

static void status(const state_machine_t *fsm)
{
    ESP_LOGI(TAG, "[STATUS] state=%s mode=%s uid=%s request=%lu", state_machine_name(fsm->state),
             fsm->direction == PARKING_ENTRY ? "ENTRY" : "EXIT", fsm->uid,
             (unsigned long)fsm->request_id);
    parking_manager_dump();
    for (unsigned i = 0; i < 2; ++i) {
        gate_ir_state_t ir = gate_ir_manager_state((parking_direction_t)i);
        ESP_LOGI(TAG, "[IR %s] raw=%s stable=%s clear_for_close=%s",
                 i == 0 ? "ENTRY" : "EXIT", ir.raw_present ? "PRESENT" : "CLEAR",
                 !ir.ready ? "SETTLING" : ir.present ? "PRESENT" : "CLEAR",
                 gate_ir_manager_clear((parking_direction_t)i, esp_timer_get_time()) ? "YES" : "NO");
    }
}

static void command(state_machine_t *fsm, app_command_t cmd, int64_t issued_us,
                    int64_t now_us)
{
    if (cmd == APP_COMMAND_HELP) {
        ESP_LOGI(TAG, "Commands: help, status, in, out (in/out require manual direction mode)");
        ESP_LOGI(TAG, "pass = VEHICLE CROSSED AND GATE AREA CLEAR; cancel = NO CROSSING AND CLEAR");
        ESP_LOGI(TAG, "Fault recovery: recover_passed or recover_cancel after physical inspection");
        ESP_LOGI(TAG, "reset_outside = mark all vehicles OUTSIDE when IDLE and all slots are free");
        return;
    }
    if (cmd == APP_COMMAND_STATUS) { status(fsm); return; }
    // Reject an assertion queued before this phase began, e.g. pass typed during OCR.
    if (issued_us < fsm->state_since_us) {
        ESP_LOGW(TAG, "[COMMAND IGNORED] command predates the current state"); return;
    }
    if (cmd == APP_COMMAND_RESET_OUTSIDE) {
        if (fsm->state != APP_STATE_IDLE) {
            ESP_LOGW(TAG, "reset_outside requires IDLE");
            return;
        }
        esp_err_t ret = parking_manager_reset_outside();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "reset_outside failed: %s", esp_err_to_name(ret));
            if (!parking_manager_storage_ready()) {
                state_machine_fault(fsm, "SAVE RESET ERROR");
            } else {
                ESP_LOGW(TAG, "reset_outside requires all slots free and no pending transaction");
            }
            return;
        }
        ESP_LOGI(TAG, "[RESET OUTSIDE] All registered vehicles are OUTSIDE");
        display_idle(fsm);
        status(fsm);
        return;
    }
    if (cmd == APP_COMMAND_ENTRY || cmd == APP_COMMAND_EXIT) {
        if (fsm->state != APP_STATE_IDLE) { ESP_LOGW(TAG, "Mode change requires IDLE"); return; }
        if (FSM_AUTO_DIRECTION) {
            ESP_LOGW(TAG, "AUTO direction enabled; in/out are not used"); return;
        }
        fsm->direction = cmd == APP_COMMAND_ENTRY ? PARKING_ENTRY : PARKING_EXIT;
        fsm->camera_id = camera_for_direction(fsm->direction);
        fsm->init_close_started = false;
        change_state(fsm, APP_STATE_INIT, now_us);
        display_manager_show("PREPARING GATE", fsm->direction == PARKING_ENTRY ? "ENTRY" : "EXIT");
        return;
    }
    if (cmd == APP_COMMAND_PASS || cmd == APP_COMMAND_CANCEL) {
        if (fsm->state != APP_STATE_WAIT_PASSAGE) {
            ESP_LOGW(TAG, "pass/cancel requires WAIT_PASSAGE"); return;
        }
        start_closing(fsm, cmd == APP_COMMAND_CANCEL, true);
        return;
    }
    if (cmd == APP_COMMAND_RECOVER_PASSED || cmd == APP_COMMAND_RECOVER_CANCEL) {
        parking_pending_t pending;
        if (fsm->state != APP_STATE_FAULT || !fsm->core_ready ||
            !parking_manager_storage_ready() || !parking_manager_get_pending(&pending)) {
            ESP_LOGW(TAG, "Recovery requires FAULT, ready hardware/storage and a pending transaction"); return;
        }
        fsm->direction = pending.direction;
        fsm->camera_id = camera_for_direction(fsm->direction);
        fsm->request_id = pending.request_id;
        memcpy(fsm->uid, pending.uid, sizeof(fsm->uid));
        if (cmd == APP_COMMAND_RECOVER_CANCEL) {
            if (pending.stage != PARKING_PENDING_AUTHORIZED) {
                ESP_LOGW(TAG, "Already PASSED: cannot cancel a recorded passage"); return;
            }
            start_closing(fsm, true, true);
        } else {
            start_closing(fsm, false, true);
        }
    }
}

void state_machine_init(state_machine_t *fsm, bool core_ready, const char *boot_error)
{
    memset(fsm, 0, sizeof(*fsm));
    fsm->state = APP_STATE_INIT;
    fsm->direction = PARKING_ENTRY;
    fsm->camera_id = FSM_ENTRY_CAMERA_ID;
    fsm->core_ready = core_ready;
    int64_t now_us = esp_timer_get_time();
    fsm->state_since_us = now_us;
    if (!core_ready) {
        state_machine_fault(fsm, boot_error ? boot_error : "INIT ERROR"); return;
    }
    parking_pending_t pending;
    if (parking_manager_get_pending(&pending)) {
        fsm->direction = pending.direction;
        fsm->camera_id = camera_for_direction(fsm->direction);
        fsm->request_id = pending.request_id;
        memcpy(fsm->uid, pending.uid, sizeof(fsm->uid));
        state_machine_fault(fsm, "RECOVER PENDING");
        ESP_LOGW(TAG, "No startup close: inspect gate/car then recover_passed or recover_cancel");
        return;
    }
    display_manager_show("INITIALIZING", "Gate / sensors");
}

void state_machine_handle(state_machine_t *fsm, const app_event_t *event)
{
    int64_t now_us = esp_timer_get_time();
    switch (event->type) {
    case APP_EVENT_CARD: new_card(fsm, event->data.uid, now_us); break;
    case APP_EVENT_OCR: ocr_result(fsm, &event->data.ocr, now_us); break;
    case APP_EVENT_COMMAND: command(fsm, event->data.command, event->created_at_us, now_us); break;
    default: break;
    }
}

void state_machine_tick(state_machine_t *fsm, int64_t now_us)
{
    barrier_manager_tick(now_us);
    gate_ir_state_t ir = gate_ir_manager_state(fsm->direction);
    bool gate_clear = gate_ir_manager_clear(fsm->direction, now_us);
    bool watching = fsm->state == APP_STATE_OPENING || fsm->state == APP_STATE_WAIT_PASSAGE ||
                    fsm->state == APP_STATE_CLOSE_DELAY || fsm->state == APP_STATE_CLOSING;
    if (watching && ir.ready && ir.present && !fsm->gate_seen_active &&
        now_us - ir.sampled_at_us <= FSM_GATE_IR_MAX_GAP_MS * 1000LL) {
        fsm->gate_seen_active = true;
        ESP_LOGI(TAG, "[PASSAGE IR] %s PRESENT after grant",
                 fsm->direction == PARKING_ENTRY ? "ENTRY" : "EXIT");
    }
    switch (fsm->state) {
    case APP_STATE_INIT: {
        bool both_clear = gate_ir_manager_clear(PARKING_ENTRY, now_us) &&
                          gate_ir_manager_clear(PARKING_EXIT, now_us);
        if (!fsm->init_close_started) {
            if (!both_clear) { break; }
            if (barrier_manager_close(PARKING_ENTRY, now_us) != ESP_OK ||
                barrier_manager_close(PARKING_EXIT, esp_timer_get_time()) != ESP_OK) {
                state_machine_fault(fsm, "INIT GATE ERROR");
                break;
            }
            fsm->init_close_started = true;
        } else if (!both_clear) {
            esp_err_t entry_ret = barrier_manager_open(PARKING_ENTRY, now_us);
            esp_err_t exit_ret = barrier_manager_open(PARKING_EXIT, esp_timer_get_time());
            state_machine_fault(fsm, entry_ret == ESP_OK && exit_ret == ESP_OK
                                      ? "INIT IR BLOCKED" : "INIT OPEN ERROR");
            break;
        }
        if (fsm->core_ready && barrier_manager_motion_done(now_us) && parking_manager_slots_ready()) {
            idle(fsm, now_us);
        }
        break;
    }
    case APP_STATE_IDLE:
        display_idle(fsm); break;
    case APP_STATE_WAIT_OCR:
        if (now_us >= fsm->deadline_us) { notice(fsm, "OCR TIMEOUT", now_us); }
        break;
    case APP_STATE_OPENING:
        if (barrier_manager_motion_done(now_us)) {
            wait_passage(fsm, now_us);
            ESP_LOGI(TAG, "[WAIT_PASSAGE] Gate IR must detect PRESENT then stable CLEAR");
        }
        break;
    case APP_STATE_WAIT_PASSAGE:
        if (fsm->gate_seen_active && gate_clear) {
            change_state(fsm, APP_STATE_CLOSE_DELAY, now_us);
            fsm->deadline_us = now_us + FSM_CLOSE_DELAY_MS * 1000LL;
            display_manager_show("CAR PASSED IR", "Close delay");
            ESP_LOGI(TAG, "[CLOSE_DELAY] IR clear; wait %u ms", (unsigned)FSM_CLOSE_DELAY_MS);
        } else if (!fsm->passage_warning && now_us >= fsm->deadline_us) {
            fsm->passage_warning = true;
            display_manager_show("PASS TIMEOUT", "Hold / check IR");
            ESP_LOGW(TAG, "Passage not confirmed; holding gate open and preserving pending");
        }
        break;
    case APP_STATE_CLOSE_DELAY:
        if (!gate_clear) {
            ESP_LOGI(TAG, "[HOLD] IR blocked/unsettled; restart close delay after CLEAR");
            wait_passage(fsm, now_us);
        } else if (now_us >= fsm->deadline_us) {
            start_closing(fsm, false, false);
        }
        break;
    case APP_STATE_CLOSING:
        if (!gate_clear) {
            reopen(fsm, now_us);
            break;
        }
        if (barrier_manager_motion_done(now_us)) {
            if (!fsm->cancelled && parking_manager_confirm_passage() != ESP_OK) {
                state_machine_fault(fsm, "SAVE PASS ERROR");
                break;
            }
            if (parking_manager_finish(fsm->cancelled) != ESP_OK) {
                state_machine_fault(fsm, "SAVE END ERROR");
            } else {
                ESP_LOGI(TAG, "[COMPLETED] uid=%s mode=%s cancelled=%u active=%u", fsm->uid,
                         fsm->direction == PARKING_ENTRY ? "ENTRY" : "EXIT",
                         (unsigned)fsm->cancelled, parking_manager_active_count());
                idle(fsm, esp_timer_get_time());
            }
        }
        break;
    case APP_STATE_NOTICE:
        if (now_us >= fsm->deadline_us) { idle(fsm, now_us); }
        break;
    case APP_STATE_FAULT: break;
    }
}
