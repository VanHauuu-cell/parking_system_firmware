#ifndef PARKING_APP_TYPES_H
#define PARKING_APP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PARKING_UID_SIZE 9
#define PARKING_PLATE_SIZE 20
#define PARKING_MAX_VEHICLES 32

typedef enum { PARKING_ENTRY = 0, PARKING_EXIT = 1 } parking_direction_t;

typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_IDLE,
    APP_STATE_WAIT_OCR,
    APP_STATE_OPENING,
    APP_STATE_WAIT_PASSAGE,
    APP_STATE_CLOSE_DELAY,
    APP_STATE_CLOSING,
    APP_STATE_NOTICE,
    APP_STATE_FAULT
} app_state_t;

typedef struct {
    char uid[PARKING_UID_SIZE];
    char plate[PARKING_PLATE_SIZE];
} vehicle_registration_t;

typedef enum {
    PARKING_PENDING_NONE = 0,
    PARKING_PENDING_AUTHORIZED,
    PARKING_PENDING_PASSED
} parking_pending_stage_t;

typedef struct {
    parking_pending_stage_t stage;
    parking_direction_t direction;
    uint32_t request_id;
    char uid[PARKING_UID_SIZE];
    char plate[PARKING_PLATE_SIZE];
} parking_pending_t;

#endif
