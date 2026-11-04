#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include "esp_err.h"

// For a buzzer module with a 3.3V-compatible digital trigger input.
esp_err_t buzzer_init(void);
esp_err_t buzzer_set(bool enabled);

#endif
