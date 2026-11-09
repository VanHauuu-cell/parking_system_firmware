#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>
#include "esp_err.h"

#define OLED_SSD1306_WIDTH 128
#define OLED_SSD1306_HEIGHT 64
#define OLED_SSD1306_PAGES 8
#define OLED_SSD1306_TEXT_COLUMNS 21

// Single-owner API for the application task. This driver owns I2C0.
// address=0 probes 0x3C/0x3D and requires exactly one responding device.
esp_err_t oled_ssd1306_init(uint8_t address);
esp_err_t oled_ssd1306_clear(void);
// One page is 8 pixels high. page=0..7, ASCII text, up to 21 characters.
// The entire page is replaced so shorter strings erase old characters.
esp_err_t oled_ssd1306_write_line(uint8_t page, const char *text);

#endif
