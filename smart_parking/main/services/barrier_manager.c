#include "barrier_manager.h"

#include "buzzer.h"
#include "esp_log.h"
#include "fsm_config.h"
#include "servo.h"
#include "system_config.h"

static const char *TAG = "BARRIER_MANAGER";
static bool initialized;
static bool buzzer_ready;
static bool buzzer_warning_shown;
static bool beep_active;
static int64_t beep_deadline_us;
static bool motion_active;
static int64_t motion_deadline_us;

static void report_buzzer_error(esp_err_t error)
{
    if (!buzzer_warning_shown) {
        ESP_LOGW(TAG, "Buzzer failed: %s; barrier control remains available",
                 esp_err_to_name(error));
        buzzer_warning_shown = true;
    }
}

esp_err_t barrier_manager_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    esp_err_t ret = servo_init();
    if (ret != ESP_OK) {
        return ret;
    }
    // PWM starts at zero duty in the existing driver. No angle is commanded.
    ret = buzzer_init();
    buzzer_ready = ret == ESP_OK;
    if (!buzzer_ready) {
        report_buzzer_error(ret);
    }
    initialized = true;
    return ESP_OK;
}

static esp_err_t move_barrier(parking_direction_t direction, bool open,
                              int64_t now_us)
{
    if ((direction != PARKING_ENTRY && direction != PARKING_EXIT) || now_us < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    servo_id_t servo = direction == PARKING_ENTRY ? SERVO_ENTRY : SERVO_EXIT;
    int angle = direction == PARKING_ENTRY
        ? (open ? FSM_ENTRY_OPEN_ANGLE : FSM_ENTRY_CLOSED_ANGLE)
        : (open ? FSM_EXIT_OPEN_ANGLE : FSM_EXIT_CLOSED_ANGLE);
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = servo_set_angle(servo, (uint8_t)angle);
    if (ret != ESP_OK) {
        return ret;
    }
    motion_active = true;
    motion_deadline_us = now_us + (int64_t)FSM_SERVO_TRAVEL_MS * 1000;
    ESP_LOGI(TAG, "%s %s; travel time is an estimate, no position feedback",
             direction == PARKING_ENTRY ? "ENTRY" : "EXIT", open ? "OPEN" : "CLOSE");
    return ESP_OK;
}

esp_err_t barrier_manager_open(parking_direction_t direction, int64_t now_us)
{
    esp_err_t ret = move_barrier(direction, true, now_us);
    if (ret != ESP_OK) {
        return ret;
    }
    if (buzzer_ready) {
        beep_active = true;
        beep_deadline_us = now_us + (int64_t)FSM_GRANT_BEEP_MS * 1000;
        esp_err_t buzzer_ret = buzzer_set(true);
        if (buzzer_ret != ESP_OK) {
            report_buzzer_error(buzzer_ret);
            barrier_manager_silence();
        }
    }
    // A buzzer error must not be reported as a failed servo command.
    return ESP_OK;
}

esp_err_t barrier_manager_close(parking_direction_t direction, int64_t now_us)
{
    esp_err_t ret = move_barrier(direction, false, now_us);
    barrier_manager_silence();
    return ret;
}

bool barrier_manager_motion_done(int64_t now_us)
{
    // This is only elapsed time after a successful PWM command, not a sensor.
    return initialized && (!motion_active || now_us >= motion_deadline_us);
}

void barrier_manager_tick(int64_t now_us)
{
    if (beep_active && now_us >= beep_deadline_us) {
        barrier_manager_silence();
    }
    if (motion_active && now_us >= motion_deadline_us) {
        motion_active = false;
    }
}

void barrier_manager_silence(void)
{
    if (!buzzer_ready) {
        return;
    }
    esp_err_t ret = buzzer_set(false);
    if (ret == ESP_OK) {
        beep_active = false;
    } else {
        // If OFF failed, keep retrying from tick() without flooding the log.
        report_buzzer_error(ret);
    }
}
