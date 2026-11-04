#include "uart_protocol.h"

#include <stdio.h>
#include <string.h>

static int build_command(char *frame, size_t capacity, const char *command,
                          uint8_t camera_id, uint32_t request_id)
{
    if (frame == NULL || capacity == 0 || camera_id == 0 || request_id == 0) {
        return -1;
    }
    int length = snprintf(frame, capacity, "<%s,%u,%lu>", command,
                          (unsigned)camera_id, (unsigned long)request_id);
    return length > 0 && (size_t)length < capacity ? length : -1;
}

int uart_protocol_build_capture_request(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id)
{
    return build_command(frame, capacity, "CAPTURE_REQ", camera_id, request_id);
}

int uart_protocol_build_watch_start(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id)
{
    return build_command(frame, capacity, "GATE_WATCH", camera_id, request_id);
}

int uart_protocol_build_watch_stop(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id)
{
    return build_command(frame, capacity, "GATE_STOP", camera_id, request_id);
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

static bool decimal_field(const char **cursor, unsigned width, uint32_t maximum,
                           uint32_t *value)
{
    const char *text = *cursor;
    uint32_t parsed = 0;
    unsigned count = 0;
    while (*text >= '0' && *text <= '9') {
        uint32_t digit = (uint32_t)(*text - '0');
        if (++count > width || parsed > (maximum - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
        ++text;
    }
    if (count == 0 || parsed == 0 || *text != ',') {
        return false;
    }
    *cursor = text + 1;
    *value = parsed;
    return true;
}

static bool decode_gate(const char *frame, uart_gate_status_t *result)
{
    static const char prefix[] = "<GATE_STATUS,";
    if (strncmp(frame, prefix, sizeof(prefix) - 1) != 0) {
        return false;
    }
    const char *cursor = frame + sizeof(prefix) - 1;
    uint32_t camera_id, request_id, sequence;
    if (!decimal_field(&cursor, 3, UINT8_MAX, &camera_id) ||
        !decimal_field(&cursor, 10, UINT32_MAX, &request_id) ||
        !decimal_field(&cursor, 10, UINT32_MAX, &sequence)) {
        return false;
    }
    bool present;
    if (strcmp(cursor, "PRESENT>") == 0) {
        present = true;
    } else if (strcmp(cursor, "CLEAR>") == 0) {
        present = false;
    } else {
        return false;
    }
    memset(result, 0, sizeof(*result));
    result->camera_id = (uint8_t)camera_id;
    result->request_id = request_id;
    result->sequence = sequence;
    result->present = present;
    return true;
}

static bool decode_message(const char *frame, uart_message_t *message)
{
    uart_gate_status_t gate;
    if (decode_gate(frame, &gate)) {
        memset(message, 0, sizeof(*message));
        message->type = UART_MESSAGE_GATE_STATUS;
        message->data.gate = gate;
        return true;
    }
    uart_ocr_result_t ocr;
    if (decode_ocr(frame, &ocr)) {
        memset(message, 0, sizeof(*message));
        message->type = UART_MESSAGE_OCR;
        message->data.ocr = ocr;
        return true;
    }
    return false;
}

bool uart_protocol_feed_message(
    uart_frame_parser_t *parser, uint8_t byte, uart_message_t *message)
{
    if (parser == NULL || message == NULL) {
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
    return decode_message(parser->frame, message);
}

bool uart_protocol_feed(
    uart_frame_parser_t *parser, uint8_t byte, uart_ocr_result_t *result)
{
    if (result == NULL) {
        return false;
    }
    uart_message_t message;
    if (uart_protocol_feed_message(parser, byte, &message) &&
        message.type == UART_MESSAGE_OCR) {
        *result = message.data.ocr;
        return true;
    }
    return false;
}
