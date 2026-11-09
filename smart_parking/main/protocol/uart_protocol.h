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

// Return frame length, or -1 on invalid arguments / insufficient space.
int uart_protocol_build_capture_request(
    char *frame, size_t capacity, uint8_t camera_id, uint32_t request_id);

// Consume one UART byte and return true when a complete OCR result is decoded.
bool uart_protocol_feed(
    uart_frame_parser_t *parser, uint8_t byte, uart_ocr_result_t *result);

#endif
