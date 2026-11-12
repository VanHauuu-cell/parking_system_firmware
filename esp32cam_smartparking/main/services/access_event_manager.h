#ifndef ACCESS_EVENT_MANAGER_H
#define ACCESS_EVENT_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t access_event_manager_start(void);
bool access_event_manager_handle_frame(const char *frame);

#endif
