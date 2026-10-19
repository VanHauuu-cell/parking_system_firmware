#include "ir_sensor.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "system_config.h"

static const char *TAG = "IR_SENSOR";

static const gpio_num_t ir_gpio[IR_SENSOR_COUNT] =
{
    GPIO_IR_SLOT_1,
    GPIO_IR_SLOT_2,
    GPIO_IR_SLOT_3,
    GPIO_IR_SLOT_4
};

esp_err_t ir_sensor_init(void)
{
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
    {
        gpio_config_t config =
        {
            .pin_bit_mask = (1ULL << ir_gpio[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        esp_err_t ret = gpio_config(&config);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure sensor %d", i + 1);
            return ret;
        }
    }

    ESP_LOGI(TAG, "IR sensors initialized");

    return ESP_OK;
}

esp_err_t ir_sensor_read(
    ir_sensor_id_t sensor_id,
    ir_sensor_state_t *state)
{
    if (sensor_id >= IR_SENSOR_COUNT || state == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int level = gpio_get_level(ir_gpio[sensor_id]);

    if (level == IR_SENSOR_ACTIVE_LEVEL)
    {
        *state = IR_SENSOR_ACTIVE;
    }
    else
    {
        *state = IR_SENSOR_INACTIVE;
    }

    return ESP_OK;
}

bool ir_sensor_is_active(ir_sensor_id_t sensor_id)
{
    ir_sensor_state_t state;

    if (ir_sensor_read(sensor_id, &state) != ESP_OK)
    {
        return false;
    }

    return state == IR_SENSOR_ACTIVE;
}