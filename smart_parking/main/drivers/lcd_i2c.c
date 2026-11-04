#include "lcd_i2c.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_config.h"

static const char *TAG = "LCD_I2C";
static i2c_master_bus_handle_t bus = NULL;
static i2c_master_dev_handle_t device = NULL;
static bool ready = false;
static uint8_t current_address = 0;

#define LCD_RS        0x01
#define LCD_EN        0x04
#define LCD_BACKLIGHT 0x08
#define LCD_TIMEOUT_MS 100

static void release_bus(void)
{
    ready = false;
    current_address = 0;
    if (device != NULL) {
        i2c_master_bus_rm_device(device);
        device = NULL;
    }
    if (bus != NULL) {
        i2c_del_master_bus(bus);
        bus = NULL;
    }
}

// SỬA ĐỔI: Tách việc bật và tắt chân EN thành các giao dịch I2C riêng
// để đảm bảo chân EN giữ mức HIGH đủ lâu cho HD44780 kịp chốt dữ liệu.
static esp_err_t write_nibble(uint8_t nibble, bool data)
{
    uint8_t base = (uint8_t)((nibble << 4) | LCD_BACKLIGHT | (data ? LCD_RS : 0));
    
    // 1. Xuất dữ liệu lên bus (EN = 0)
    uint8_t data_en_low = base;
    esp_err_t ret = i2c_master_transmit(device, &data_en_low, 1, LCD_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(1);

    // 2. Kéo EN lên HIGH
    uint8_t data_en_high = base | LCD_EN;
    ret = i2c_master_transmit(device, &data_en_high, 1, LCD_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(2); // Giữ xung EN HIGH ít nhất 2µs

    // 3. Phản hồi chốt: Kéo EN xuống LOW
    ret = i2c_master_transmit(device, &data_en_low, 1, LCD_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;
    esp_rom_delay_us(50); // Thời gian chờ lệnh thực thi cơ bản

    return ESP_OK;
}

static esp_err_t write_byte(uint8_t value, bool data)
{
    esp_err_t ret = write_nibble(value >> 4, data);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = write_nibble(value & 0x0F, data);
    if (ret == ESP_OK) {
        // Lệnh Clear (0x01) và Return Home (0x02) cần 2ms để thực thi
        if (!data && (value == 0x01 || value == 0x02)) {
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }
    return ret;
}

static esp_err_t find_address(uint8_t *address)
{
    unsigned found = 0;
    for (unsigned candidate = 0x20; candidate <= 0x3F; ++candidate) {
        if (candidate > 0x27 && candidate < 0x38) {
            continue;
        }
        esp_err_t ret = i2c_master_probe(bus, candidate, LCD_TIMEOUT_MS);
        if (ret != ESP_OK) {
            continue;
        }
        *address = (uint8_t)candidate;
        ESP_LOGI(TAG, "I2C device found: 0x%02X", candidate);
        ++found;
    }
    if (found == 0) {
        ESP_LOGE(TAG, "No PCF8574 found; check power, level shifting and SDA/SCL");
        return ESP_ERR_NOT_FOUND;
    }
    if (found != 1) {
        ESP_LOGE(TAG, "Multiple devices found; set the LCD address explicitly");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t lcd_i2c_init(uint8_t address)
{
    if (ready) {
        return address == 0 || address == current_address ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (address != 0 && !((address >= 0x20 && address <= 0x27) ||
                          (address >= 0x38 && address <= 0x3F))) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_LCD_SDA,
        .scl_io_num = GPIO_LCD_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus);
    if (ret != ESP_OK) {
        return ret;
    }
    if (address == 0) {
        ret = find_address(&address);
    } else {
        ret = i2c_master_probe(bus, address, LCD_TIMEOUT_MS);
    }
    if (ret != ESP_OK) {
        release_bus();
        return ret;
    }
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus, &device_config, &device);
    if (ret != ESP_OK) {
        release_bus();
        return ret;
    }

    // Quy trình Reset 4-bit tiêu chuẩn cho HD44780
    vTaskDelay(pdMS_TO_TICKS(100)); // Chờ nguồn LCD ổn định (>40ms)
    
    ret = write_nibble(0x03, false);
    if (ret != ESP_OK) { goto fail; }
    vTaskDelay(pdMS_TO_TICKS(5));   // Chờ >4.1ms

    ret = write_nibble(0x03, false);
    if (ret != ESP_OK) { goto fail; }
    vTaskDelay(pdMS_TO_TICKS(1));   // Chờ >100us

    ret = write_nibble(0x03, false);
    if (ret != ESP_OK) { goto fail; }
    vTaskDelay(pdMS_TO_TICKS(1));

    ret = write_nibble(0x02, false); // Chuyển sang 4-bit mode
    if (ret != ESP_OK) { goto fail; }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Thiết lập cấu hình khởi tạo LCD
    ret = write_byte(0x28, false); // 4-bit mode, 2 dòng, font 5x8
    if (ret != ESP_OK) { goto fail; }

    ret = write_byte(0x08, false); // Tắt màn hình
    if (ret != ESP_OK) { goto fail; }

    ret = write_byte(0x01, false); // Xóa màn hình (tốn 2ms)
    if (ret != ESP_OK) { goto fail; }

    ret = write_byte(0x06, false); // Entry mode: Tự động tăng con trỏ
    if (ret != ESP_OK) { goto fail; }

    ret = write_byte(0x0C, false); // Bật màn hình, tắt con trỏ
    if (ret != ESP_OK) { goto fail; }

    ready = true;
    current_address = address;
    ESP_LOGI(TAG, "LCD1602 ready: address=0x%02X SDA=%d SCL=%d",
             address, GPIO_LCD_SDA, GPIO_LCD_SCL);
    return ESP_OK;

fail:
    release_bus();
    return ret;
}

esp_err_t lcd_i2c_clear(void)
{
    return ready ? write_byte(0x01, false) : ESP_ERR_INVALID_STATE;
}

esp_err_t lcd_i2c_write_line(uint8_t row, const char *text)
{
    if (!ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (row > 1 || text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    char line[16];
    memset(line, ' ', sizeof(line));
    size_t length = strlen(text);
    if (length > sizeof(line)) { length = sizeof(line); }
    for (size_t i = 0; i < length; ++i) {
        unsigned char c = (unsigned char)text[i];
        line[i] = c >= 0x20 && c <= 0x7E ? (char)c : ' ';
    }
    esp_err_t ret = write_byte((uint8_t)(0x80 | (row == 0 ? 0x00 : 0x40)), false);
    if (ret != ESP_OK) { return ret; }
    for (size_t i = 0; i < sizeof(line); ++i) {
        ret = write_byte((uint8_t)line[i], true);
        if (ret != ESP_OK) { return ret; }
    }
    return ESP_OK;
}