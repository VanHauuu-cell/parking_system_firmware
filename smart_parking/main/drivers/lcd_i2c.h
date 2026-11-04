#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include "esp_err.h"

// LCD1602 + PCF8574, common mapping:
// P0=RS P1=RW P2=EN P3=backlight P4..P7=D4..D7.
// address=0 scans PCF8574/PCF8574A addresses; exactly one device must respond.
esp_err_t lcd_i2c_init(uint8_t address);
esp_err_t lcd_i2c_clear(void);
// row=0 or 1; ASCII only; truncate/pad to 16 characters.
esp_err_t lcd_i2c_write_line(uint8_t row, const char *text);

#endif
