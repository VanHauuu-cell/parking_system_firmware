#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    UART_LINK_ENTRY = 0,
    UART_LINK_EXIT,
    UART_LINK_COUNT
} uart_link_t;

// Initialize both links: ENTRY=UART2, EXIT=UART1. UART0 remains the console.
esp_err_t uart_driver_init(void);
esp_err_t uart_driver_write_link(uart_link_t link, const uint8_t *data, size_t length);
int uart_driver_read_link(uart_link_t link, uint8_t *buffer, size_t length,
                          uint32_t timeout_ms);

// Compatibility wrappers for earlier single-camera code: ENTRY link only.
esp_err_t uart_driver_write(const uint8_t *data, size_t length);
int uart_driver_read(uint8_t *buffer, size_t length, uint32_t timeout_ms);
void uart_driver_flush(void);

#endif
