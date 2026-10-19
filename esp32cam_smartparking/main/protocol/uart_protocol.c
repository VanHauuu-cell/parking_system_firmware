#include "uart_protocol.h"

#include <stdio.h>
#include <string.h>

#include "uart_driver.h"

esp_err_t uart_protocol_send_plate_result(
    uint8_t camera_id,
    uint32_t event_id,
    const char *plate,
    uint8_t confidence
)
{
    char frame[UART_MAX_FRAME_SIZE];

    int length = snprintf(
        frame,
        sizeof(frame),
        "<PLATE_RESULT,%d,%lu,%s,%d>",
        camera_id,
        (unsigned long)event_id,
        plate,
        confidence
    );

    if (length <= 0 || length >= sizeof(frame))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return uart_driver_write(
        (const uint8_t *)frame,
        length
    );
}