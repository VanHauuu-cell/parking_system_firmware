#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>
#include "esp_err.h"

#define UART_FRAME_START '<'
#define UART_FRAME_END   '>'

#define UART_MAX_FRAME_SIZE 128

typedef enum
{
    UART_MSG_HEARTBEAT,
    UART_MSG_PLATE_RESULT,
    UART_MSG_CAMERA_STATUS,
    UART_MSG_ACK
} uart_message_type_t;

typedef struct
{
    uint8_t camera_id;
    uint32_t event_id;
    char plate[20];
    uint8_t confidence;
} uart_plate_result_t;

esp_err_t uart_protocol_send_plate_result(
    uint8_t camera_id,
    uint32_t event_id,
    const char *plate,
    uint8_t confidence
);

#endif