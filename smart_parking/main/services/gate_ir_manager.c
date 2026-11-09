#include "gate_ir_manager.h"

#include "esp_log.h"
#include "fsm_config.h"
#include "ir_sensor.h"

_Static_assert(FSM_GATE_IR_DEBOUNCE_SAMPLES > 0 &&
               FSM_GATE_IR_DEBOUNCE_SAMPLES <= 255, "Invalid gate debounce count");
_Static_assert(FSM_GATE_IR_POLL_MS > 0 &&
               FSM_GATE_IR_MAX_GAP_MS > FSM_GATE_IR_POLL_MS, "Invalid gate timing");

static const char *TAG = "GATE_IR";
static gate_ir_state_t states[2];
static uint8_t samples[2];
static bool read_error[2];
static int64_t last_poll_us = -1;

void gate_ir_manager_poll(int64_t now_us)
{
    if (last_poll_us >= 0 && now_us - last_poll_us < FSM_GATE_IR_POLL_MS * 1000LL) {
        return;
    }
    bool gap = last_poll_us < 0 ||
               now_us - last_poll_us > FSM_GATE_IR_MAX_GAP_MS * 1000LL;
    last_poll_us = now_us;
    const ir_sensor_id_t sensors[] = {IR_SENSOR_GATE_ENTRY, IR_SENSOR_GATE_EXIT};
    for (unsigned i = 0; i < 2; ++i) {
        gate_ir_state_t *state = &states[i];
        ir_sensor_state_t raw;
        esp_err_t ret = ir_sensor_read(sensors[i], &raw);
        if (ret != ESP_OK) {
            if (!read_error[i]) {
                ESP_LOGE(TAG, "%s read failed: %s", i == 0 ? "ENTRY" : "EXIT",
                         esp_err_to_name(ret));
            }
            read_error[i] = true;
            state->ready = false;
            state->clear_since_us = 0;
            samples[i] = 0;
            continue;
        }
        read_error[i] = false;
        bool present = raw == IR_SENSOR_ACTIVE;
        if (gap || samples[i] == 0 || present != state->raw_present) {
            samples[i] = 1;
            state->ready = false;
            state->clear_since_us = 0;
        } else if (samples[i] < FSM_GATE_IR_DEBOUNCE_SAMPLES) {
            ++samples[i];
        }
        state->raw_present = present;
        state->sampled_at_us = now_us;
        if (samples[i] >= FSM_GATE_IR_DEBOUNCE_SAMPLES && !state->ready) {
            state->ready = true;
            state->present = present;
            state->clear_since_us = present ? 0 : now_us;
            ESP_LOGI(TAG, "%s %s", i == 0 ? "ENTRY" : "EXIT",
                     present ? "PRESENT" : "CLEAR");
        }
    }
}

gate_ir_state_t gate_ir_manager_state(parking_direction_t direction)
{
    if (direction != PARKING_ENTRY && direction != PARKING_EXIT) {
        return (gate_ir_state_t){0};
    }
    return states[direction];
}

bool gate_ir_manager_clear(parking_direction_t direction, int64_t now_us)
{
    gate_ir_state_t state = gate_ir_manager_state(direction);
    return state.ready && !state.present && !state.raw_present &&
           state.clear_since_us > 0 && now_us >= state.sampled_at_us &&
           now_us - state.sampled_at_us <= FSM_GATE_IR_MAX_GAP_MS * 1000LL &&
           now_us - state.clear_since_us >= FSM_GATE_IR_CLEAR_STABLE_MS * 1000LL;
}
