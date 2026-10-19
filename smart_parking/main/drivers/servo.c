#include "servo.h"

#include "driver/ledc.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "system_config.h"

static const char *TAG = "SERVO";

static const gpio_num_t servo_gpio[SERVO_COUNT] =
{
    GPIO_SERVO_ENTRY,
    GPIO_SERVO_EXIT
};

static uint32_t angle_to_pulse(uint8_t angle)
{
    return SERVO_MIN_PULSE_US +
           ((uint32_t)angle *
            (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
           SERVO_MAX_ANGLE;
}

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000 / SERVO_PWM_FREQ_HZ;
    const uint32_t max_duty = (1UL << 16) - 1;

    return (pulse_us * max_duty) / period_us;
}

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_config =
    {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&timer_config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure PWM timer");
        return ret;
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ledc_channel_config_t channel_config =
        {
            .gpio_num = servo_gpio[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };

        ret = ledc_channel_config(&channel_config);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "Failed to configure servo channel %d",
                i
            );

            return ret;
        }
    }

    ESP_LOGI(TAG, "Servo driver initialized");

    return ESP_OK;
}

esp_err_t servo_set_angle(
    servo_id_t servo,
    uint8_t angle)
{
    if (servo >= SERVO_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (angle > SERVO_MAX_ANGLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pulse_us = angle_to_pulse(angle);
    uint32_t duty = pulse_to_duty(pulse_us);

    esp_err_t ret = ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        (ledc_channel_t)servo,
        duty
    );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        (ledc_channel_t)servo
    );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_LOGI(
        TAG,
        "Servo %d -> %u deg (%lu us)",
        servo,
        angle,
        pulse_us
    );

    return ESP_OK;
}