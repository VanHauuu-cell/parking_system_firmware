#ifndef PARKING_MANAGER_H
#define PARKING_MANAGER_H

#include "app_types.h"
#include "esp_err.h"

// Single-owner API: call only from the application's main task.
// No automatic NVS erase on initialization/storage errors.
esp_err_t parking_manager_init(void);
bool parking_manager_storage_ready(void);
const vehicle_registration_t *parking_manager_find(const char *uid);
bool parking_manager_is_inside(const char *uid);
unsigned parking_manager_active_count(void);
bool parking_manager_get_pending(parking_pending_t *pending);

// Debounce the parking-slot sensors. free_count is the minimum of physical
// empty slots and capacity left after active entries / pending admission.
esp_err_t parking_manager_poll_slots(int64_t now_us);
bool parking_manager_slots_ready(void);
unsigned parking_manager_physical_free(void);
unsigned parking_manager_free_count(void);

// Persist BEFORE sending an open command. Only one pending transaction exists.
esp_err_t parking_manager_begin(const char *uid, parking_direction_t direction,
                                uint32_t request_id);
// Atomically update the inside ledger and mark the SAME pending transaction
// PASSED. Repeating this call on PASSED must not count a second entry/exit.
esp_err_t parking_manager_confirm_passage(void);
// Call after closing. cancelled=true is only allowed for AUTHORIZED, following
// an operator confirmation that no vehicle crossed and the gate area is clear.
esp_err_t parking_manager_finish(bool cancelled);
// Reset the inside flags only when the ledger has no pending transaction and
// the slot sensors have settled with every slot free.
esp_err_t parking_manager_reset_outside(void);
void parking_manager_dump(void);

#endif
