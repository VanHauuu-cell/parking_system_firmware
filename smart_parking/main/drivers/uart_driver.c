#include "uart_driver.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "gpio_config.h"

#define UART_PORT           UART_NUM_2

#define UART_BAUD_RATE      115200

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

static const char *TAG = "UART_DRIVER";

esp_err_t uart_driver_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;

    ret = uart_driver_install(
        UART_PORT,
        UART_RX_BUFFER_SIZE,
        UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return ret;
    }

    ret = uart_param_config(
        UART_PORT,
        &uart_config
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed");
        return ret;
    }

    ret = uart_set_pin(
        UART_PORT,
        GPIO_CAM_ENTRY_TX,
        GPIO_CAM_ENTRY_RX,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return ret;
    }

    ESP_LOGI(
        TAG,
        "UART initialized: UART2 TX=%d RX=%d baud=%d",
        GPIO_CAM_ENTRY_TX,
        GPIO_CAM_ENTRY_RX,
        UART_BAUD_RATE
    );

    return ESP_OK;
}

esp_err_t uart_driver_write(
    const uint8_t *data,
    size_t length
)
{
    if (data == NULL || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(
        UART_PORT,
        data,
        length
    );

    if (written < 0)
    {
        return ESP_FAIL;
    }

    if ((size_t)written != length)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

int uart_driver_read(
    uint8_t *buffer,
    size_t length,
    uint32_t timeout_ms
)
{
    if (buffer == NULL || length == 0)
    {
        return -1;
    }

    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    return uart_read_bytes(
        UART_PORT,
        buffer,
        length,
        timeout_ticks
    );
}

void uart_driver_flush(void)
{
    uart_flush_input(UART_PORT);
}