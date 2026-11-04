#ifndef GATE_WATCH_MANAGER_H
#define GATE_WATCH_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

// Call after HTTP/UART initialization, before starting the sole UART reader.
esp_err_t gate_watch_manager_start(void);

// Called by ocr_manager's UART RX task, never by a second UART reader.
// True means a valid GATE_WATCH/GATE_STOP frame was handled or ignored by ID.
bool gate_watch_manager_handle_frame(const char *frame);

#endif
