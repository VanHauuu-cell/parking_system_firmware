#ifndef UART_MANAGER_H
#define UART_MANAGER_H

#include "uart_protocol.h"
#include "esp_err.h"

esp_err_t uart_manager_init(void);
// Route camera ID to its physical ENTRY/EXIT UART configured in fsm_config.h.
esp_err_t uart_manager_request(uint8_t camera_id, uint32_t request_id);
// Nonblocking round-robin poll. Both links have independent framing state.
// False also means the bounded byte budget was reached, so poll again next tick.
bool uart_manager_poll(uart_ocr_result_t *result);

#endif
