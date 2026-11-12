#ifndef PARKING_APPROVAL_LOG_H
#define PARKING_APPROVAL_LOG_H

#include "app_types.h"
#include "esp_err.h"

// One owner, the application task. parking_manager_init() initializes NVS first.
esp_err_t approval_log_init(void);
bool approval_log_ready(void);
// Keep space for both an approval and its possible later cancellation.
bool approval_log_can_grant(void);
unsigned approval_log_pending_count(void);
bool approval_log_was_cancelled(uint8_t camera_id, uint32_t request_id, const char *uid);
esp_err_t approval_log_record_grant(uint8_t camera_id, uint32_t request_id,
                                  const char *uid, const char *plate,
                                  int64_t approved_at_ms);
esp_err_t approval_log_record_cancel(uint8_t camera_id, uint32_t request_id,
                                   const char *uid);
bool approval_log_peek(approval_event_t *event);
esp_err_t approval_log_ack(uint8_t camera_id, const char *event_id);

#endif
