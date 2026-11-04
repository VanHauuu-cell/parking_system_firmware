#ifndef PARKING_STATE_MACHINE_H
#define PARKING_STATE_MACHINE_H

#include "app_events.h"

typedef struct {
    app_state_t state;
    parking_direction_t direction;
    bool core_ready;
    bool cancelled;
    bool passage_warning;
    uint8_t camera_id;
    bool camera_seen_present;
    bool camera_present;
    bool camera_status_valid;
    bool closing_from_camera;
    uint32_t last_camera_sequence;
    unsigned clear_samples;
    int64_t last_camera_status_us;
    int64_t clear_since_us;
    int64_t state_since_us;
    int64_t deadline_us;
    uint32_t request_id;
    char uid[PARKING_UID_SIZE];
} state_machine_t;

void state_machine_init(state_machine_t *fsm, bool core_ready, const char *boot_error);
void state_machine_handle(state_machine_t *fsm, const app_event_t *event);
void state_machine_tick(state_machine_t *fsm, int64_t now_us);
void state_machine_fault(state_machine_t *fsm, const char *reason);
const char *state_machine_name(app_state_t state);

#endif
