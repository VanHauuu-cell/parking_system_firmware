#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "esp_err.h"

typedef enum
{
    SERVO_ENTRY = 0,
    SERVO_EXIT,
    SERVO_COUNT
} servo_id_t;

esp_err_t servo_init(void);

esp_err_t servo_set_angle(
    servo_id_t servo,
    uint8_t angle
);

#endif