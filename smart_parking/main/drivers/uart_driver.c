#include "uart_driver.h"

#include <limits.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "gpio_config.h"

#define UART_BAUD_RATE 115200
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

static const char *TAG = "UART_DRIVER";
static const uart_port_t ports[UART_LINK_COUNT] = {UART_NUM_2, UART_NUM_1};
static const int tx_pins[UART_LINK_COUNT] = {GPIO_CAM_ENTRY_TX, GPIO_CAM_EXIT_TX};
static const int rx_pins[UART_LINK_COUNT] = {GPIO_CAM_ENTRY_RX, GPIO_CAM_EXIT_RX};
static bool initialized[UART_LINK_COUNT];

static bool valid_link(uart_link_t link)
{
    return link == UART_LINK_ENTRY || link == UART_LINK_EXIT;
}

esp_err_t uart_driver_init(void)
{
    const uart_config_t config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    bool installed_this_call[UART_LINK_COUNT] = {false};
    esp_err_t ret = ESP_OK;
    for (unsigned link = 0; link < UART_LINK_COUNT; ++link) {
        if (initialized[link]) {
            continue;
        }
        ret = uart_driver_install(ports[link], UART_RX_BUFFER_SIZE,
                                  UART_TX_BUFFER_SIZE, 0, NULL, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UART%d install failed: %s", (int)ports[link],
                     esp_err_to_name(ret));
            goto fail;
        }
        installed_this_call[link] = true;
        ret = uart_param_config(ports[link], &config);
        if (ret != ESP_OK) {
            goto fail;
        }
        ret = uart_set_pin(ports[link], tx_pins[link], rx_pins[link],
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            goto fail;
        }
        initialized[link] = true;
        ESP_LOGI(TAG, "%s UART%d TX=%d RX=%d baud=%d",
                 link == UART_LINK_ENTRY ? "ENTRY" : "EXIT", (int)ports[link],
                 tx_pins[link], rx_pins[link], UART_BAUD_RATE);
    }
    return ESP_OK;

fail:
    // Only release drivers installed by this call, never someone else's UART.
    for (unsigned link = 0; link < UART_LINK_COUNT; ++link) {
        if (installed_this_call[link]) {
            esp_err_t cleanup_ret = uart_driver_delete(ports[link]);
            if (cleanup_ret != ESP_OK) {
                ESP_LOGW(TAG, "UART%d cleanup failed: %s", (int)ports[link],
                         esp_err_to_name(cleanup_ret));
            }
            initialized[link] = false;
        }
    }
    ESP_LOGE(TAG, "UART initialization failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t uart_driver_write_link(uart_link_t link, const uint8_t *data, size_t length)
{
    if (!valid_link(link) || data == NULL || length == 0 || length > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized[link]) {
        return ESP_ERR_INVALID_STATE;
    }
    int written = uart_write_bytes(ports[link], data, length);
    return written >= 0 && (size_t)written == length ? ESP_OK : ESP_FAIL;
}

int uart_driver_read_link(uart_link_t link, uint8_t *buffer, size_t length,
                          uint32_t timeout_ms)
{
    if (!valid_link(link) || buffer == NULL || length == 0 || length > INT_MAX ||
            !initialized[link]) {
        return -1;
    }
    return uart_read_bytes(ports[link], buffer, length, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t uart_driver_write(const uint8_t *data, size_t length)
{
    return uart_driver_write_link(UART_LINK_ENTRY, data, length);
}

int uart_driver_read(uint8_t *buffer, size_t length, uint32_t timeout_ms)
{
    return uart_driver_read_link(UART_LINK_ENTRY, buffer, length, timeout_ms);
}

void uart_driver_flush(void)
{
    if (initialized[UART_LINK_ENTRY]) {
        uart_flush_input(ports[UART_LINK_ENTRY]);
    }
}
