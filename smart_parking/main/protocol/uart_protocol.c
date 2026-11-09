#include "uart_protocol.h"

#include <stdio.h>
#include <string.h>

int uart_protocol_build_capture_request(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id)
{
    if (frame == NULL || capacity == 0 || camera_id == 0 || request_id == 0) {
        return -1;
    }
    int length = snprintf(frame, capacity, "<CAPTURE_REQ,%u,%lu>",
                          (unsigned)camera_id, (unsigned long)request_id);
    return length > 0 && (size_t)length < capacity ? length : -1;
}

static bool decode_ocr(const char *frame, uart_ocr_result_t *result)
{
    unsigned camera_id = 0;
    unsigned confidence = 0;
    unsigned long long request_id = 0;
    char plate[20] = {0};
    char reason[24] = {0};
    int end = -1;

    int count = sscanf(frame,
        "<PLATE_RESULT,%3u,%10llu,%19[A-Z0-9],%3u>%n",
        &camera_id, &request_id, plate, &confidence, &end);

    if (count == 4 && end >= 0 && (size_t)end == strlen(frame) &&
        camera_id >= 1 && camera_id <= 255 &&
        request_id >= 1 && request_id <= UINT32_MAX && confidence <= 100) {
        memset(result, 0, sizeof(*result));
        result->camera_id = (uint8_t)camera_id;
        result->request_id = (uint32_t)request_id;
        result->success = true;
        result->confidence = (uint8_t)confidence;
        memcpy(result->plate, plate, strlen(plate) + 1);
        return true;
    }

    end = -1;
    count = sscanf(frame, "<PLATE_ERROR,%3u,%10llu,%23[A-Z_]>%n",
                   &camera_id, &request_id, reason, &end);
    if (count != 3 || end < 0 || (size_t)end != strlen(frame) ||
        camera_id < 1 || camera_id > 255 ||
        request_id < 1 || request_id > UINT32_MAX ||
        (strcmp(reason, "NO_PLATE") != 0 &&
         strcmp(reason, "OCR_FAILED") != 0 && strcmp(reason, "ERROR") != 0)) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->camera_id = (uint8_t)camera_id;
    result->request_id = (uint32_t)request_id;
    memcpy(result->reason, reason, strlen(reason) + 1);
    return true;
}

bool uart_protocol_feed(
    uart_frame_parser_t *parser, uint8_t byte, uart_ocr_result_t *result)
{
    if (parser == NULL || result == NULL) {
        return false;
    }
    char c = (char)byte;
    if (c == '<') {
        parser->collecting = true;
        parser->used = 0;
    }
    if (!parser->collecting) {
        return false;
    }
    if (parser->used >= sizeof(parser->frame) - 1) {
        parser->collecting = false;
        parser->used = 0;
        return false;
    }
    parser->frame[parser->used++] = c;
    if (c != '>') {
        return false;
    }
    parser->frame[parser->used] = '\0';
    parser->collecting = false;
    parser->used = 0;
    return decode_ocr(parser->frame, result);
}
