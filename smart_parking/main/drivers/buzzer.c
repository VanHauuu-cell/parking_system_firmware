#include "buzzer.h"

#include "driver/gpio.h"
#include "gpio_config.h"
#include "system_config.h"

static bool initialized = false;

esp_err_t buzzer_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    const int off_level = BUZZER_ACTIVE_LEVEL ? 0 : 1;
    // Preload the OFF output latch before switching the pin to output mode.
    esp_err_t ret = gpio_set_level(GPIO_BUZZER, off_level);
    if (ret != ESP_OK) { return ret; }
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << GPIO_BUZZER,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&config);
    if (ret != ESP_OK) { return ret; }
    ret = gpio_set_level(GPIO_BUZZER, off_level);
    initialized = ret == ESP_OK;
    return ret;
}

esp_err_t buzzer_set(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    int level = enabled ? BUZZER_ACTIVE_LEVEL : (BUZZER_ACTIVE_LEVEL ? 0 : 1);
    return gpio_set_level(GPIO_BUZZER, level);
}
