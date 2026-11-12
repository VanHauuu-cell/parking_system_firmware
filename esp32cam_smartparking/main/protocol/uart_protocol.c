#include "uart_protocol.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "uart_driver.h"

bool uart_protocol_parse_capture_request(
    const char *frame, uint8_t *camera_id, uint32_t *request_id)
{
    if (frame == NULL || camera_id == NULL || request_id == NULL) {
        return false;
    }
    unsigned camera = 0;
    unsigned long long id = 0;
    int end = -1;
    int fields = sscanf(frame, "<CAPTURE_REQ,%3u,%10llu>%n", &camera, &id, &end);
    if (fields != 2 || end < 0 || (size_t)end != strlen(frame) ||
        camera < 1 || camera > 255 || id < 1 || id > UINT32_MAX) {
        return false;
    }
    *camera_id = (uint8_t)camera;
    *request_id = (uint32_t)id;
    return true;
}

static esp_err_t send_frame(const char *frame, int length)
{
    if (length <= 0 || length >= UART_MAX_FRAME_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    return uart_driver_write((const uint8_t *)frame, (size_t)length);
}

esp_err_t uart_protocol_send_plate_result(
    uint8_t camera_id, uint32_t request_id, const char *plate, uint8_t confidence,
    int64_t server_time_ms)
{
    if (camera_id == 0 || request_id == 0 || plate == NULL || confidence > 100 ||
            server_time_ms < PARKING_TIMESTAMP_MIN_MS ||
            server_time_ms > PARKING_TIMESTAMP_MAX_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t size = strlen(plate);
    if (size == 0 || size >= 20) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < size; ++i) {
        char c = plate[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    char frame[UART_MAX_FRAME_SIZE];
    int length = snprintf(frame, sizeof(frame), "<PLATE_RESULT,%u,%lu,%s,%u,%" PRId64 ">",
                          (unsigned)camera_id, (unsigned long)request_id,
                          plate, (unsigned)confidence, server_time_ms);
    return send_frame(frame, length);
}

esp_err_t uart_protocol_send_plate_error(
    uint8_t camera_id, uint32_t request_id, const char *reason)
{
    if (camera_id == 0 || request_id == 0 || reason == NULL ||
        (strcmp(reason, "NO_PLATE") != 0 && strcmp(reason, "OCR_FAILED") != 0 &&
         strcmp(reason, "ERROR") != 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    char frame[UART_MAX_FRAME_SIZE];
    int length = snprintf(frame, sizeof(frame), "<PLATE_ERROR,%u,%lu,%s>",
                          (unsigned)camera_id, (unsigned long)request_id, reason);
    return send_frame(frame, length);
}
