#include "display_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "fsm_config.h"
#include "oled_ssd1306.h"
#include "system_config.h"

static const char *TAG = "DISPLAY_MANAGER";
static bool init_attempted;
static bool available;
static bool cached[2];
static char last_line[2][OLED_SSD1306_TEXT_COLUMNS + 1];

static void format_line(char output[OLED_SSD1306_TEXT_COLUMNS + 1], const char *text)
{
    memset(output, ' ', OLED_SSD1306_TEXT_COLUMNS);
    output[OLED_SSD1306_TEXT_COLUMNS] = '\0';
    if (text == NULL) {
        return;
    }
    for (unsigned i = 0; i < OLED_SSD1306_TEXT_COLUMNS && text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char)text[i];
        output[i] = c >= 0x20 && c <= 0x7E ? (char)c : '?';
    }
}

esp_err_t display_manager_init(void)
{
    if (init_attempted) {
        return ESP_OK;
    }
    init_attempted = true;
    esp_err_t ret = oled_ssd1306_init(FSM_OLED_ADDRESS);
    if (ret == ESP_OK) {
        ret = oled_ssd1306_write_line(0, "SMART PARKING");
    }
    available = ret == ESP_OK;
    if (!available) {
        ESP_LOGW(TAG, "OLED unavailable: %s; continuing with serial logs", esp_err_to_name(ret));
    }
    // The screen is optional for access control, as in the existing service.
    return ESP_OK;
}

void display_manager_show(const char *line1, const char *line2)
{
    if (!available) {
        return;
    }
    const char *lines[] = {line1, line2};
    const uint8_t pages[] = {3, 5};
    for (unsigned row = 0; row < 2; ++row) {
        char formatted[OLED_SSD1306_TEXT_COLUMNS + 1];
        format_line(formatted, lines[row]);
        if (cached[row] && memcmp(last_line[row], formatted, sizeof(formatted)) == 0) {
            continue;
        }
        esp_err_t ret = oled_ssd1306_write_line(pages[row], formatted);
        if (ret != ESP_OK) {
            available = false;
            ESP_LOGW(TAG, "OLED write failed: %s; continuing with serial logs", esp_err_to_name(ret));
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
    char line[OLED_SSD1306_TEXT_COLUMNS + 1];
    snprintf(line, sizeof(line), "%s FREE:%u/%u",
             direction == PARKING_ENTRY ? "ENTRY" : "EXIT", free_count,
             (unsigned)PARKING_SLOT_COUNT);
    display_manager_show(line, "Tap RFID card");
}
