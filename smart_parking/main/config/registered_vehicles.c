#include "registered_vehicles.h"

// EXAMPLE UIDs: replace them with your real 4-byte card UIDs from the MAIN log.
// Enter the plate in uppercase without spaces, dots or hyphens.
const vehicle_registration_t registered_vehicles[] = {
    {"72B3B25B", "60K33286"},
    {"B32E0F05", "43A90518"},
    {"CA5DCA35", "29B67124"},
    {"7A2B4E35", "51H24873"},
    {"555A40E0", "18C71450"},
};

const size_t registered_vehicle_count =
    sizeof(registered_vehicles) / sizeof(registered_vehicles[0]);

_Static_assert(sizeof(registered_vehicles) / sizeof(registered_vehicles[0]) <=
               PARKING_MAX_VEHICLES, "Too many registered vehicles");
