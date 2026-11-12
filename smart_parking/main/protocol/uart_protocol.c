#include "uart_protocol.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

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

int uart_protocol_build_access_event(
    char *frame, size_t capacity, const approval_event_t *event)
{
    if (frame == NULL || capacity == 0 || event == NULL) { return -1; }
    int length = snprintf(frame, capacity,
        "<ACCESS_EVENT,%u,%lu,%s,%s,%c,%s,%s,%" PRId64 ">",
        (unsigned)event->camera_id, (unsigned long)event->request_id,
        event->event_id, event->approval_id, event->kind,
        event->uid, event->plate, event->approved_at_ms);
    return length > 0 && (size_t)length < capacity ? length : -1;
}

static bool decode_ocr(const char *frame, uart_ocr_result_t *result)
{
    unsigned camera_id = 0;
    unsigned confidence = 0;
    unsigned long long request_id = 0;
    char plate[20] = {0};
    char reason[24] = {0};
    long long server_time_ms = 0;
    int end = -1;

    int count = sscanf(frame,
        "<PLATE_RESULT,%3u,%10llu,%19[A-Z0-9],%3u,%13lld>%n",
        &camera_id, &request_id, plate, &confidence, &server_time_ms, &end);

    // Let the FSM report TIME NOT READY when a previous CAM image is running.
    bool legacy = false;
    if (count != 5 || end < 0 || (size_t)end != strlen(frame)) {
        end = -1;
        count = sscanf(frame, "<PLATE_RESULT,%3u,%10llu,%19[A-Z0-9],%3u>%n",
                       &camera_id, &request_id, plate, &confidence, &end);
        legacy = count == 4 && end >= 0 && (size_t)end == strlen(frame);
        server_time_ms = 0;
    }

    if ((legacy || count == 5) && end >= 0 && (size_t)end == strlen(frame) &&
        camera_id >= 1 && camera_id <= 255 &&
        request_id >= 1 && request_id <= UINT32_MAX && confidence <= 100 &&
        (legacy || (server_time_ms >= APPROVAL_TIME_MIN_MS &&
                    server_time_ms <= APPROVAL_TIME_MAX_MS))) {
        memset(result, 0, sizeof(*result));
        result->camera_id = (uint8_t)camera_id;
        result->request_id = (uint32_t)request_id;
        result->success = true;
        result->confidence = (uint8_t)confidence;
        result->server_time_ms = (int64_t)server_time_ms;
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

static bool decode_message(const char *frame, uart_message_t *message)
{
    unsigned camera_id = 0;
    char event_id[APPROVAL_EVENT_ID_SIZE] = {0};
    int end = -1;
    int count = sscanf(frame, "<ACCESS_ACK,%3u,%16[0-9A-F]>%n",
                       &camera_id, event_id, &end);
    if (count == 2 && end >= 0 && (size_t)end == strlen(frame) &&
        camera_id >= 1 && camera_id <= 255 && strlen(event_id) == 16) {
        memset(message, 0, sizeof(*message));
        message->type = UART_MESSAGE_ACCESS_ACK;
        message->data.ack.camera_id = (uint8_t)camera_id;
        memcpy(message->data.ack.event_id, event_id, sizeof(event_id));
        return true;
    }
    memset(message, 0, sizeof(*message));
    message->type = UART_MESSAGE_OCR;
    return decode_ocr(frame, &message->data.ocr);
}

bool uart_protocol_feed_message(
    uart_frame_parser_t *parser, uint8_t byte, uart_message_t *message)
{
    if (parser == NULL || message == NULL) { return false; }
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
