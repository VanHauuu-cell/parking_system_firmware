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
    bool gate_seen_active;
    bool manual_closing;
    bool init_close_started;
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
