#ifndef BARRIER_MANAGER_H
#define BARRIER_MANAGER_H

#include "app_types.h"
#include "esp_err.h"

// Initializes PWM/buzzer only; does not move either barrier.
esp_err_t barrier_manager_init(void);
esp_err_t barrier_manager_open(parking_direction_t direction, int64_t now_us);
esp_err_t barrier_manager_close(parking_direction_t direction, int64_t now_us);
bool barrier_manager_motion_done(int64_t now_us);
void barrier_manager_tick(int64_t now_us);
void barrier_manager_silence(void);

#endif
