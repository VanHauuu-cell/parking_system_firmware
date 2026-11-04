#ifndef PARKING_APP_EVENTS_H
#define PARKING_APP_EVENTS_H

#include "app_types.h"
#include "uart_protocol.h"

typedef enum {
    APP_COMMAND_HELP = 0,
    APP_COMMAND_STATUS,
    APP_COMMAND_ENTRY,
    APP_COMMAND_EXIT,
    APP_COMMAND_PASS,
    APP_COMMAND_CANCEL,
    APP_COMMAND_RECOVER_PASSED,
    APP_COMMAND_RECOVER_CANCEL,
    APP_COMMAND_RESET_OUTSIDE
} app_command_t;

typedef enum {
    APP_EVENT_CARD = 0,
    APP_EVENT_OCR,
    APP_EVENT_GATE_STATUS,
    APP_EVENT_PASSAGE_CONFIRMED,
    APP_EVENT_COMMAND
} app_event_type_t;

typedef struct {
    app_event_type_t type;
    int64_t created_at_us;
    union {
        char uid[PARKING_UID_SIZE];
        uart_ocr_result_t ocr;
        uart_gate_status_t gate;
        struct { uint8_t camera_id; uint32_t request_id; } passage;
        app_command_t command;
    } data;
} app_event_t;

#endif
