#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "esp_err.h"
#include <stdbool.h>

typedef enum
{
    IR_SENSOR_1 = 0,
    IR_SENSOR_2,
    IR_SENSOR_3,
    IR_SENSOR_4,
    IR_SENSOR_COUNT
} ir_sensor_id_t;

typedef enum
{
    IR_SENSOR_INACTIVE = 0,
    IR_SENSOR_ACTIVE = 1
} ir_sensor_state_t;

esp_err_t ir_sensor_init(void);

esp_err_t ir_sensor_read(
    ir_sensor_id_t sensor_id,
    ir_sensor_state_t *state
);

bool ir_sensor_is_active(ir_sensor_id_t sensor_id);

#endif