#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "app_types.h"
#include "esp_err.h"

esp_err_t display_manager_init(void);
// ASCII lines, cached to avoid repeatedly clearing/rewriting the LCD.
void display_manager_show(const char *line1, const char *line2);
void display_manager_idle(parking_direction_t direction, unsigned free_count);

#endif
