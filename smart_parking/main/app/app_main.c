#include "app_main.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/uart_vfs.h"

#include "app_events.h"
#include "state_machine.h"
#include "fsm_config.h"
#include "mfrc522.h"
#include "parking_manager.h"
#include "barrier_manager.h"
#include "display_manager.h"
#include "uart_manager.h"

static const char *TAG = "PARKING_APP";
static QueueHandle_t command_queue;
static bool started;

static bool enqueue(const app_event_t *event)
{
    return command_queue != NULL && xQueueSend(command_queue, event, 0) == pdTRUE;
}

bool parking_app_confirm_passage(uint8_t camera_id, uint32_t request_id)
{
    if (camera_id == 0 || request_id == 0) { return false; }
    app_event_t event = {
        .type = APP_EVENT_PASSAGE_CONFIRMED,
        .created_at_us = esp_timer_get_time(),
        .data.passage = {.camera_id = camera_id, .request_id = request_id}
    };
    return enqueue(&event);
}

static bool read_uid(char text[PARKING_UID_SIZE])
{
    // The current driver can write up to 16 bytes even for short responses.
    uint8_t tag_type[16] = {0};
    uint8_t serial[16] = {0};
    // Preserve the REQA/anticoll helper already used successfully in main.c.
    mfrc522_clear_bit_mask(MFRC522_REG_TX_CONTROL, 0x03);
    vTaskDelay(pdMS_TO_TICKS(10));
    mfrc522_antenna_on();
    vTaskDelay(pdMS_TO_TICKS(10));
    if (mfrc522_request(PICC_CMD_REQA, tag_type) != MFRC522_STATUS_OK ||
        mfrc522_anticoll(serial) != MFRC522_STATUS_OK || serial[0] == 0x88) {
        return false;
    }
    snprintf(text, PARKING_UID_SIZE, "%02X%02X%02X%02X",
             serial[0], serial[1], serial[2], serial[3]);
    return true;
}

#if FSM_ENABLE_SERIAL_COMMANDS
static void submit_command(const char *line)
{
    static const struct { const char *name; app_command_t command; } commands[] = {
        {"help", APP_COMMAND_HELP}, {"status", APP_COMMAND_STATUS},
        {"in", APP_COMMAND_ENTRY}, {"out", APP_COMMAND_EXIT},
        {"pass", APP_COMMAND_PASS}, {"cancel", APP_COMMAND_CANCEL},
        {"recover_passed", APP_COMMAND_RECOVER_PASSED},
        {"recover_cancel", APP_COMMAND_RECOVER_CANCEL},
        {"reset_outside", APP_COMMAND_RESET_OUTSIDE},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        if (strcmp(line, commands[i].name) == 0) {
            app_event_t event = {.type = APP_EVENT_COMMAND,
                                  .created_at_us = esp_timer_get_time(),
                                  .data.command = commands[i].command};
            if (!enqueue(&event)) { ESP_LOGW(TAG, "Command queue full; command was not accepted"); }
            else { ESP_LOGI(TAG, "[COMMAND] %s", line); }
            return;
        }
    }
    ESP_LOGW(TAG, "Unknown command '%s'; type help then Enter", line);
}

static void console_task(void *arg)
{
    (void)arg;
    // Existing default UART0 VFS reads without a UART0 driver installation.
    // Handle both CR/LF ourselves instead of waiting for CRLF look-ahead.
#if defined(CONFIG_ESP_CONSOLE_UART_NUM) && CONFIG_ESP_CONSOLE_UART_NUM >= 0
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
#endif
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) { (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); }
    char line[40];
    size_t used = 0;
    bool overflow = false;
    while (1) {
        char buffer[32];
        int length = (int)read(STDIN_FILENO, buffer, sizeof(buffer));
        for (int i = 0; i < length; ++i) {
            char c = buffer[i];
            if (c == '\r' || c == '\n') {
                if (!overflow && used > 0) { line[used] = '\0'; submit_command(line); }
                else if (overflow) { ESP_LOGW(TAG, "Command too long; discarded"); }
                used = 0;
                overflow = false;
            } else if (c == '\b' || c == 0x7f) {
                if (!overflow && used > 0) { --used; }
            } else if (c >= 0x20 && c <= 0x7e) {
                if (used + 1 < sizeof(line) && !overflow) { line[used++] = c; }
                else { overflow = true; }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

static void check_init(esp_err_t ret, const char *name, bool *ready, const char **reason)
{
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s init failed: %s", name, esp_err_to_name(ret));
        if (*ready) { *reason = name; }
        *ready = false;
    }
}

static void application_task(void *arg)
{
    (void)arg;
    // A missing LCD/buzzer remains a log-only warning; servo, ledger, RFID and
    // UART failures prevent grants. This task is the only FSM/storage owner.
    (void)display_manager_init();
    bool core_ready = true;
    const char *boot_error = NULL;
    esp_err_t ret = parking_manager_init();
    bool parking_ready = ret == ESP_OK;
    check_init(ret, "PARKING / NVS", &core_ready, &boot_error);
    ret = barrier_manager_init();
    check_init(ret, "SERVO", &core_ready, &boot_error);
    ret = uart_manager_init();
    bool uart_ready = ret == ESP_OK;
    check_init(ret, "UART", &core_ready, &boot_error);
    ret = mfrc522_init();
    bool rfid_ready = ret == ESP_OK;
    if (rfid_ready) {
        uint8_t version = mfrc522_get_version();
        ESP_LOGI(TAG, "RC522 version=0x%02X", version);
        if (version == 0 || version == 0xff) { ret = ESP_FAIL; rfid_ready = false; }
    }
    check_init(ret, "RFID", &core_ready, &boot_error);

    state_machine_t fsm;
    state_machine_init(&fsm, core_ready, boot_error);
#if FSM_ENABLE_SERIAL_COMMANDS
    if (xTaskCreate(console_task, "parking_console", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Console task unavailable; camera events can still operate the FSM");
    } else { ESP_LOGI(TAG, "Type help / status then Enter in Serial Monitor"); }
#endif
    bool card_latched = false;
    int64_t last_seen_us = 0;
    int64_t next_scan_us = 0;
    while (1) {
        // Drain camera observations before a close deadline is evaluated.
        // OCR handlers separately reject results past their own deadline.
        if (parking_ready) {
            ret = parking_manager_poll_slots(esp_timer_get_time());
            if (ret != ESP_OK && fsm.state != APP_STATE_FAULT) {
                state_machine_fault(&fsm, "SLOT READ ERROR");
            }
        }
        if (uart_ready) {
            for (unsigned i = 0; i < 8; ++i) {
                uart_message_t message;
                // false can also mean a byte budget was consumed while more
                // buffered bytes remain. Keep all eight bounded drain attempts.
                if (!uart_manager_poll_message(&message)) { continue; }
                app_event_t event = {.created_at_us = esp_timer_get_time()};
                if (message.type == UART_MESSAGE_OCR) {
                    event.type = APP_EVENT_OCR;
                    event.data.ocr = message.data.ocr;
                } else {
                    event.type = APP_EVENT_GATE_STATUS;
                    event.data.gate = message.data.gate;
                }
                state_machine_handle(&fsm, &event);
            }
        }
        for (unsigned i = 0; i < 8; ++i) {
            app_event_t event;
            if (xQueueReceive(command_queue, &event, 0) != pdTRUE) { break; }
            state_machine_handle(&fsm, &event);
        }
        state_machine_tick(&fsm, esp_timer_get_time());
        int64_t now_us = esp_timer_get_time();
        if (rfid_ready && now_us >= next_scan_us) {
            char uid[PARKING_UID_SIZE];
            bool present = read_uid(uid);
            now_us = esp_timer_get_time();
            next_scan_us = now_us + (int64_t)FSM_RFID_SCAN_MS * 1000;
            if (present) {
                last_seen_us = now_us;
                if (!card_latched) {
                    card_latched = true;
                    app_event_t event = {.type = APP_EVENT_CARD, .created_at_us = now_us};
                    memcpy(event.data.uid, uid, sizeof(uid));
                    ESP_LOGI(TAG, "[RFID] uid=%s", uid);
                    state_machine_handle(&fsm, &event);
                }
            } else if (now_us - last_seen_us >= (int64_t)FSM_CARD_RELEASE_MS * 1000) {
                card_latched = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool parking_app_start(void)
{
    if (started) { return true; }
    command_queue = xQueueCreate(8, sizeof(app_event_t));
    if (command_queue == NULL) { return false; }
    if (xTaskCreate(application_task, "parking_app", 8192, NULL, 5, NULL) != pdPASS) {
        vQueueDelete(command_queue);
        command_queue = NULL;
        return false;
    }
    started = true;
    return true;
}
