#ifndef PARKING_APPLICATION_H
#define PARKING_APPLICATION_H

#include <stdbool.h>
#include <stdint.h>

// Starts one task which owns the FSM, services and hardware polling.
bool parking_app_start(void);

// Optional future task-context hook for an actual passage sensor. It must mean
// a vehicle crossed AND the barrier area is clear; not merely "sensor inactive".
// Not ISR-safe. The camera heuristic uses GATE_STATUS and its own delay instead.
bool parking_app_confirm_passage(uint8_t camera_id, uint32_t request_id);

#endif
