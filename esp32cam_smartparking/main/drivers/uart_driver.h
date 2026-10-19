#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t uart_driver_init(void);

esp_err_t uart_driver_write(
    const uint8_t *data,
    size_t length
);

int uart_driver_read(
    uint8_t *buffer,
    size_t length,
    uint32_t timeout_ms
);

void uart_driver_flush(void);

#endif