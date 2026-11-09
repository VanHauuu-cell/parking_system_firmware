#ifndef PARKING_APPLICATION_H
#define PARKING_APPLICATION_H

#include <stdbool.h>
#include <stdint.h>

// Starts one task which owns the FSM, services and hardware polling.
bool parking_app_start(void);

#endif
