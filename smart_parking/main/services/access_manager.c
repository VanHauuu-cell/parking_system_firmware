#include "access_manager.h"

#include <string.h>
#include "parking_manager.h"

access_decision_t access_manager_check(const char *uid, const char *plate,
                                      parking_direction_t direction)
{
    if (!parking_manager_storage_ready() ||
        (direction != PARKING_ENTRY && direction != PARKING_EXIT)) {
        return ACCESS_NOT_READY;
    }
    parking_pending_t pending;
    if (parking_manager_get_pending(&pending)) {
        return ACCESS_BUSY;
    }
    const vehicle_registration_t *vehicle = parking_manager_find(uid);
    if (vehicle == NULL) {
        return ACCESS_UNKNOWN_CARD;
    }
    if (plate == NULL || strncmp(vehicle->plate, plate, PARKING_PLATE_SIZE) != 0) {
        return ACCESS_WRONG_PLATE;
    }
    bool inside = parking_manager_is_inside(uid);
    if (direction == PARKING_ENTRY) {
        if (inside) {
            return ACCESS_ALREADY_INSIDE;
        }
        if (!parking_manager_slots_ready()) {
            return ACCESS_NOT_READY;
        }
        if (parking_manager_free_count() == 0) {
            return ACCESS_FULL;
        }
    } else if (!inside) {
        return ACCESS_NOT_INSIDE;
    }
    return ACCESS_ALLOWED;
}

const char *access_manager_reason(access_decision_t decision)
{
    switch (decision) {
    case ACCESS_ALLOWED:        return "ACCESS OK";
    case ACCESS_UNKNOWN_CARD:   return "UNKNOWN CARD";
    case ACCESS_WRONG_PLATE:    return "WRONG PLATE";
    case ACCESS_ALREADY_INSIDE: return "ALREADY INSIDE";
    case ACCESS_NOT_INSIDE:     return "NOT INSIDE";
    case ACCESS_FULL:           return "PARKING FULL";
    case ACCESS_NOT_READY:      return "NOT READY";
    case ACCESS_BUSY:           return "SYSTEM BUSY";
    default:                    return "ACCESS ERROR";
    }
}
