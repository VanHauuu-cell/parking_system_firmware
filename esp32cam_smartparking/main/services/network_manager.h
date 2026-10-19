#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

esp_err_t network_manager_init(void);

esp_err_t network_manager_connect(void);

bool network_manager_is_connected(void);

#endif