#ifndef GATE_IR_MANAGER_H
#define GATE_IR_MANAGER_H

#include "app_types.h"

typedef struct {
    bool ready;
    bool present;
    bool raw_present;
    int64_t sampled_at_us;
    int64_t clear_since_us;
} gate_ir_state_t;

// The application task owns this service. ir_sensor_init() runs first.
void gate_ir_manager_poll(int64_t now_us);
gate_ir_state_t gate_ir_manager_state(parking_direction_t direction);
bool gate_ir_manager_clear(parking_direction_t direction, int64_t now_us);

#endif
