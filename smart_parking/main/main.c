#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "uart_driver.h"

static const char *TAG = "CAM_UART_TEST";

void app_main(void)
{
    ESP_ERROR_CHECK(uart_driver_init());

    uint8_t buffer[128];

    while (1)
    {
        int length = uart_driver_read(
            buffer,
            sizeof(buffer) - 1,
            1000
        );

        if (length > 0)
        {
            buffer[length] = '\0';

            ESP_LOGI(
                TAG,
                "Received: %s",
                (char *)buffer
            );
        }
    }
}