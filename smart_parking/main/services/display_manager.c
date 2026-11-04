#include "display_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "fsm_config.h"
#include "lcd_i2c.h"
#include "system_config.h"

static const char *TAG = "DISPLAY_MANAGER";
static bool init_attempted;
static bool available;
static bool cached[2];
static char last_line[2][17];

static void format_line(char output[17], const char *text)
{
    memset(output, ' ', 16);
    output[16] = '\0';
    if (text == NULL) {
        return;
    }
    for (size_t i = 0; i < 16 && text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char)text[i];
        output[i] = c >= 0x20 && c <= 0x7e ? (char)c : ' ';
    }
}

esp_err_t display_manager_init(void)
{
    if (init_attempted) {
        return ESP_OK;
    }
    init_attempted = true;
    esp_err_t ret = lcd_i2c_init(FSM_LCD_ADDRESS);
    available = ret == ESP_OK;
    if (!available) {
        ESP_LOGW(TAG, "LCD unavailable: %s; continuing with serial logs",
                 esp_err_to_name(ret));
    }
    // The LCD is optional. Its absence must not put access control in FAULT.
    return ESP_OK;
}

void display_manager_show(const char *line1, const char *line2)
{
    if (!available) {
        return;
    }
    const char *lines[2] = {line1, line2};
    for (unsigned row = 0; row < 2; ++row) {
        char formatted[17];
        format_line(formatted, lines[row]);
        if (cached[row] && memcmp(last_line[row], formatted, sizeof(formatted)) == 0) {
            continue;
        }
        esp_err_t ret = lcd_i2c_write_line((uint8_t)row, formatted);
        if (ret != ESP_OK) {
            available = false;
            ESP_LOGW(TAG, "LCD write failed: %s; continuing with serial logs",
                     esp_err_to_name(ret));
            return;
        }
        memcpy(last_line[row], formatted, sizeof(formatted));
        cached[row] = true;
    }
}

void display_manager_idle(parking_direction_t direction, unsigned free_count)
{
    if (direction != PARKING_ENTRY && direction != PARKING_EXIT) {
        display_manager_show("INVALID MODE", "Check mode");
        return;
    }
    if (free_count > PARKING_SLOT_COUNT) {
        free_count = PARKING_SLOT_COUNT;
    }
    char line[17];
    snprintf(line, sizeof(line), "%s FREE:%u/%u",
             direction == PARKING_ENTRY ? "ENTRY" : "EXIT", free_count,
             (unsigned)PARKING_SLOT_COUNT);
    display_manager_show(line, "Tap RFID card");
}
