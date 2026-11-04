#ifndef ACCESS_MANAGER_H
#define ACCESS_MANAGER_H

#include "app_types.h"

typedef enum {
    ACCESS_ALLOWED = 0,
    ACCESS_UNKNOWN_CARD,
    ACCESS_WRONG_PLATE,
    ACCESS_ALREADY_INSIDE,
    ACCESS_NOT_INSIDE,
    ACCESS_FULL,
    ACCESS_NOT_READY,
    ACCESS_BUSY
} access_decision_t;

access_decision_t access_manager_check(const char *uid, const char *plate,
                                      parking_direction_t direction);
const char *access_manager_reason(access_decision_t decision);

#endif
