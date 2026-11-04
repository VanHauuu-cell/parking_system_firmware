#ifndef OCR_MANAGER_H
#define OCR_MANAGER_H

#include "esp_err.h"

// Call after uart_driver_init() and http_server_manager_start().
esp_err_t ocr_manager_start(void);

#endif
