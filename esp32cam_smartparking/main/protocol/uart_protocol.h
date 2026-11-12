#ifndef CAM_UART_PROTOCOL_H
#define CAM_UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define UART_MAX_FRAME_SIZE 128
#define PARKING_TIMESTAMP_MIN_MS INT64_C(1577836800000)
#define PARKING_TIMESTAMP_MAX_MS INT64_C(4102444800000)

bool uart_protocol_parse_capture_request(
    const char *frame, uint8_t *camera_id, uint32_t *request_id);

esp_err_t uart_protocol_send_plate_result(
    uint8_t camera_id, uint32_t request_id,
    const char *plate, uint8_t confidence, int64_t server_time_ms);

esp_err_t uart_protocol_send_plate_error(
    uint8_t camera_id, uint32_t request_id, const char *reason);

#endif
