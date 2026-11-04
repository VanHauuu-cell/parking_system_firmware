#ifndef CAM_UART_PROTOCOL_H
#define CAM_UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define UART_MAX_FRAME_SIZE 128

bool uart_protocol_parse_capture_request(
    const char *frame, uint8_t *camera_id, uint32_t *request_id);

esp_err_t uart_protocol_send_plate_result(
    uint8_t camera_id, uint32_t request_id,
    const char *plate, uint8_t confidence);

esp_err_t uart_protocol_send_plate_error(
    uint8_t camera_id, uint32_t request_id, const char *reason);

#endif
