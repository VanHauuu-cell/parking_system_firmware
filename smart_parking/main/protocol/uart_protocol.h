#ifndef PARKING_UART_PROTOCOL_H
#define PARKING_UART_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PARKING_UART_FRAME_SIZE 128

typedef struct {
    char frame[PARKING_UART_FRAME_SIZE];
    size_t used;
    bool collecting;
} uart_frame_parser_t;

typedef struct {
    uint8_t camera_id;
    uint32_t request_id;
    bool success;
    char plate[20];
    uint8_t confidence;
    char reason[24];
} uart_ocr_result_t;

typedef struct {
    uint8_t camera_id;
    uint32_t request_id;
    uint32_t sequence;
    bool present;
} uart_gate_status_t;

typedef enum {
    UART_MESSAGE_OCR = 0,
    UART_MESSAGE_GATE_STATUS
} uart_message_type_t;

typedef struct {
    uart_message_type_t type;
    union {
        uart_ocr_result_t ocr;
        uart_gate_status_t gate;
    } data;
} uart_message_t;

// Return frame length, or -1 on invalid arguments / insufficient space.
int uart_protocol_build_capture_request(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id);
int uart_protocol_build_watch_start(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id);
int uart_protocol_build_watch_stop(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id);

// One shared byte framer; use this API when both message types are needed.
bool uart_protocol_feed_message(
    uart_frame_parser_t *parser, uint8_t byte, uart_message_t *message);

// Compatibility API: consumes all frames but returns only OCR results.
bool uart_protocol_feed(
    uart_frame_parser_t *parser, uint8_t byte, uart_ocr_result_t *result);

#endif
