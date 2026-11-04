#include "parking_manager.h"

#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "fsm_config.h"
#include "ir_sensor.h"
#include "registered_vehicles.h"
#include "system_config.h"

#define LEDGER_MAGIC   0x504B4631UL
#define LEDGER_VERSION 1UL
#define LEDGER_KEY     "ledger"

typedef struct {
    char uid[PARKING_UID_SIZE];
    char plate[PARKING_PLATE_SIZE];
    uint8_t inside;
} stored_vehicle_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    stored_vehicle_t vehicles[PARKING_MAX_VEHICLES];
    parking_pending_t pending;
} parking_blob_t;

_Static_assert(PARKING_SLOT_COUNT == 4, "This example manages four parking slots");
_Static_assert(IR_SENSOR_COUNT == PARKING_SLOT_COUNT, "IR/slot count mismatch");
_Static_assert(FSM_IR_DEBOUNCE_SAMPLES > 0 && FSM_IR_DEBOUNCE_SAMPLES <= 255,
               "Invalid IR debounce sample count");
_Static_assert(FSM_IR_POLL_MS > 0, "Invalid IR polling interval");

static const char *TAG = "PARKING_MANAGER";
static bool s_init_attempted;
static bool s_registry_valid;
static bool s_storage_ready;
static bool s_ir_initialized;
static bool s_nvs_open;
static nvs_handle_t s_nvs;

// This service has one owner. Static working copies avoid a large task stack.
static parking_blob_t s_current;
static parking_blob_t s_candidate;
static parking_blob_t s_loaded;
static bool s_slots_ready;
static bool s_slot_candidate[PARKING_SLOT_COUNT];
static bool s_slot_occupied[PARKING_SLOT_COUNT];
static bool s_slot_settled[PARKING_SLOT_COUNT];
static uint8_t s_slot_samples[PARKING_SLOT_COUNT];
static int64_t s_last_poll_us = -1;

static size_t bounded_length(const char *text, size_t capacity)
{
    if (text == NULL) {
        return capacity;
    }
    size_t length = 0;
    while (length < capacity && text[length] != '\0') {
        ++length;
    }
    return length;
}

static bool valid_uid(const char *uid)
{
    if (bounded_length(uid, PARKING_UID_SIZE) != 8) {
        return false;
    }
    for (size_t i = 0; i < 8; ++i) {
        if (!((uid[i] >= '0' && uid[i] <= '9') ||
              (uid[i] >= 'A' && uid[i] <= 'F'))) {
            return false;
        }
    }
    return true;
}

static bool valid_plate(const char *plate)
{
    size_t length = bounded_length(plate, PARKING_PLATE_SIZE);
    if (length == 0 || length >= PARKING_PLATE_SIZE) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        if (!((plate[i] >= '0' && plate[i] <= '9') ||
              (plate[i] >= 'A' && plate[i] <= 'Z'))) {
            return false;
        }
    }
    return true;
}

static int registration_index(const char *uid)
{
    for (size_t i = 0; i < registered_vehicle_count; ++i) {
        if (strcmp(registered_vehicles[i].uid, uid) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static int stored_index(const parking_blob_t *blob, const char *uid)
{
    for (uint32_t i = 0; i < blob->count; ++i) {
        if (strcmp(blob->vehicles[i].uid, uid) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static esp_err_t validate_registry(void)
{
    if (registered_vehicle_count > PARKING_MAX_VEHICLES) {
        return ESP_ERR_INVALID_SIZE;
    }
    for (size_t i = 0; i < registered_vehicle_count; ++i) {
        const vehicle_registration_t *item = &registered_vehicles[i];
        if (!valid_uid(item->uid) || !valid_plate(item->plate)) {
            ESP_LOGE(TAG, "Invalid registration at index %u", (unsigned)i);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = 0; j < i; ++j) {
            if (strcmp(item->uid, registered_vehicles[j].uid) == 0 ||
                strcmp(item->plate, registered_vehicles[j].plate) == 0) {
                ESP_LOGE(TAG, "Duplicate UID or plate at index %u", (unsigned)i);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

static bool validate_blob(const parking_blob_t *blob)
{
    if (blob->magic != LEDGER_MAGIC || blob->version != LEDGER_VERSION ||
        blob->count > PARKING_MAX_VEHICLES) {
        return false;
    }
    unsigned active = 0;
    for (uint32_t i = 0; i < blob->count; ++i) {
        const stored_vehicle_t *item = &blob->vehicles[i];
        if (!valid_uid(item->uid) || !valid_plate(item->plate) || item->inside > 1) {
            return false;
        }
        active += item->inside;
        for (uint32_t j = 0; j < i; ++j) {
            if (strcmp(item->uid, blob->vehicles[j].uid) == 0 ||
                strcmp(item->plate, blob->vehicles[j].plate) == 0) {
                return false;
            }
        }
    }
    if (active > PARKING_SLOT_COUNT) {
        return false;
    }
    const parking_pending_t *pending = &blob->pending;
    if (pending->stage == PARKING_PENDING_NONE) {
        return pending->direction == PARKING_ENTRY && pending->request_id == 0 &&
               pending->uid[0] == '\0' && pending->plate[0] == '\0';
    }
    if ((pending->stage != PARKING_PENDING_AUTHORIZED &&
         pending->stage != PARKING_PENDING_PASSED) ||
        (pending->direction != PARKING_ENTRY && pending->direction != PARKING_EXIT) ||
        pending->request_id == 0 || !valid_uid(pending->uid) ||
        !valid_plate(pending->plate)) {
        return false;
    }
    int index = stored_index(blob, pending->uid);
    if (index < 0 || strcmp(blob->vehicles[index].plate, pending->plate) != 0) {
        return false;
    }
    bool expected_inside = pending->direction == PARKING_EXIT;
    if (pending->stage == PARKING_PENDING_PASSED) {
        expected_inside = !expected_inside;
    }
    if ((blob->vehicles[index].inside != 0) != expected_inside) {
        return false;
    }
    // An authorized entry has already reserved the remaining place.
    return !(pending->stage == PARKING_PENDING_AUTHORIZED &&
             pending->direction == PARKING_ENTRY && active >= PARKING_SLOT_COUNT);
}

static void prepare_registry_blob(parking_blob_t *blob)
{
    memset(blob, 0, sizeof(*blob));
    blob->magic = LEDGER_MAGIC;
    blob->version = LEDGER_VERSION;
    blob->count = (uint32_t)registered_vehicle_count;
    for (size_t i = 0; i < registered_vehicle_count; ++i) {
        memcpy(blob->vehicles[i].uid, registered_vehicles[i].uid, PARKING_UID_SIZE);
        memcpy(blob->vehicles[i].plate, registered_vehicles[i].plate, PARKING_PLATE_SIZE);
    }
}

static esp_err_t persist_candidate(void)
{
    if (!s_nvs_open || !validate_blob(&s_candidate)) {
        s_storage_ready = false;
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = nvs_set_blob(s_nvs, LEDGER_KEY, &s_candidate, sizeof(s_candidate));
    if (ret == ESP_OK) {
        ret = nvs_commit(s_nvs);
    }
    if (ret != ESP_OK) {
        s_storage_ready = false;
        ESP_LOGE(TAG, "Ledger write failed: %s; grants/mutations locked until restart",
                 esp_err_to_name(ret));
        return ret;
    }
    s_current = s_candidate;
    return ESP_OK;
}

static esp_err_t reconcile_registry(void)
{
    prepare_registry_blob(&s_candidate);
    for (uint32_t i = 0; i < s_loaded.count; ++i) {
        const stored_vehicle_t *old = &s_loaded.vehicles[i];
        int index = registration_index(old->uid);
        bool matches = index >= 0 &&
                       strcmp(registered_vehicles[index].plate, old->plate) == 0;
        if (old->inside && !matches) {
            ESP_LOGE(TAG, "Active registration changed/removed: UID=%s plate=%s",
                     old->uid, old->plate);
            return ESP_ERR_INVALID_STATE;
        }
        if (matches) {
            s_candidate.vehicles[index].inside = old->inside;
        }
    }
    if (s_loaded.pending.stage != PARKING_PENDING_NONE) {
        int index = registration_index(s_loaded.pending.uid);
        if (index < 0 || strcmp(registered_vehicles[index].plate,
                                s_loaded.pending.plate) != 0) {
            ESP_LOGE(TAG, "Pending registration changed/removed: UID=%s", s_loaded.pending.uid);
            return ESP_ERR_INVALID_STATE;
        }
        s_candidate.pending = s_loaded.pending;
    }
    if (!validate_blob(&s_candidate)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (memcmp(&s_candidate, &s_loaded, sizeof(s_candidate)) == 0) {
        s_current = s_candidate;
        return ESP_OK;
    }
    return persist_candidate();
}

esp_err_t parking_manager_init(void)
{
    if (s_init_attempted) {
        return ESP_ERR_INVALID_STATE;
    }
    s_init_attempted = true;
    esp_err_t ret = validate_registry();
    if (ret != ESP_OK) {
        return ret;
    }
    s_registry_valid = true;
    ret = ir_sensor_init();
    if (ret != ESP_OK) {
        return ret;
    }
    s_ir_initialized = true;
    ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s; no automatic erase", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_open("park_fsm", NVS_READWRITE, &s_nvs);
    if (ret != ESP_OK) {
        return ret;
    }
    s_nvs_open = true;
    size_t length = 0;
    ret = nvs_get_blob(s_nvs, LEDGER_KEY, NULL, &length);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        prepare_registry_blob(&s_candidate);
        ret = persist_candidate();
    } else if (ret == ESP_OK) {
        if (length != sizeof(s_loaded)) {
            ESP_LOGE(TAG, "Ledger size mismatch; no erase/migration performed");
            return ESP_ERR_INVALID_SIZE;
        }
        ret = nvs_get_blob(s_nvs, LEDGER_KEY, &s_loaded, &length);
        if (ret == ESP_OK && !validate_blob(&s_loaded)) {
            ESP_LOGE(TAG, "Invalid ledger/version; no erase performed");
            return ESP_ERR_INVALID_STATE;
        }
        if (ret == ESP_OK) {
            ret = reconcile_registry();
        }
    }
    if (ret != ESP_OK) {
        s_storage_ready = false;
        return ret;
    }
    s_storage_ready = true;
    ESP_LOGI(TAG, "Ledger ready: %u registered, %u inside, pending=%u",
             (unsigned)s_current.count, parking_manager_active_count(),
             (unsigned)s_current.pending.stage);
    return ESP_OK;
}

bool parking_manager_storage_ready(void)
{
    return s_storage_ready;
}

const vehicle_registration_t *parking_manager_find(const char *uid)
{
    if (!s_registry_valid || !valid_uid(uid)) {
        return NULL;
    }
    int index = registration_index(uid);
    return index < 0 ? NULL : &registered_vehicles[index];
}

bool parking_manager_is_inside(const char *uid)
{
    if (!valid_uid(uid)) {
        return false;
    }
    int index = stored_index(&s_current, uid);
    return index >= 0 && s_current.vehicles[index].inside != 0;
}

unsigned parking_manager_active_count(void)
{
    unsigned count = 0;
    for (uint32_t i = 0; i < s_current.count; ++i) {
        count += s_current.vehicles[i].inside != 0;
    }
    return count;
}

bool parking_manager_get_pending(parking_pending_t *pending)
{
    if (pending != NULL) {
        *pending = s_current.pending;
    }
    return s_current.pending.stage != PARKING_PENDING_NONE;
}

esp_err_t parking_manager_poll_slots(int64_t now_us)
{
    if (!s_ir_initialized || now_us < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_last_poll_us >= 0 && now_us - s_last_poll_us < FSM_IR_POLL_MS * 1000LL) {
        return ESP_OK;
    }
    s_last_poll_us = now_us;
    bool samples[PARKING_SLOT_COUNT];
    for (unsigned i = 0; i < PARKING_SLOT_COUNT; ++i) {
        ir_sensor_state_t state;
        esp_err_t ret = ir_sensor_read((ir_sensor_id_t)i, &state);
        if (ret != ESP_OK) {
            s_slots_ready = false;
            memset(s_slot_settled, 0, sizeof(s_slot_settled));
            memset(s_slot_samples, 0, sizeof(s_slot_samples));
            return ret;
        }
        samples[i] = state == IR_SENSOR_ACTIVE;
    }
    bool all_settled = true;
    for (unsigned i = 0; i < PARKING_SLOT_COUNT; ++i) {
        if (s_slot_samples[i] == 0 || samples[i] != s_slot_candidate[i]) {
            s_slot_candidate[i] = samples[i];
            s_slot_samples[i] = 1;
            s_slot_settled[i] = false;
        } else if (s_slot_samples[i] < FSM_IR_DEBOUNCE_SAMPLES) {
            ++s_slot_samples[i];
        }
        if (s_slot_samples[i] >= FSM_IR_DEBOUNCE_SAMPLES) {
            s_slot_occupied[i] = s_slot_candidate[i];
            s_slot_settled[i] = true;
        }
        all_settled = all_settled && s_slot_settled[i];
    }
    s_slots_ready = all_settled;
    return ESP_OK;
}

bool parking_manager_slots_ready(void)
{
    return s_slots_ready;
}

unsigned parking_manager_physical_free(void)
{
    if (!s_slots_ready) {
        return 0;
    }
    unsigned count = 0;
    for (unsigned i = 0; i < PARKING_SLOT_COUNT; ++i) {
        count += !s_slot_occupied[i];
    }
    return count;
}

unsigned parking_manager_free_count(void)
{
    if (!s_storage_ready || !s_slots_ready) {
        return 0;
    }
    unsigned used = parking_manager_active_count();
    if (s_current.pending.stage == PARKING_PENDING_AUTHORIZED &&
        s_current.pending.direction == PARKING_ENTRY) {
        ++used;
    }
    unsigned available = used < PARKING_SLOT_COUNT ? PARKING_SLOT_COUNT - used : 0;
    unsigned physical = parking_manager_physical_free();
    return physical < available ? physical : available;
}

esp_err_t parking_manager_begin(const char *uid, parking_direction_t direction,
                                uint32_t request_id)
{
    if (!valid_uid(uid) || request_id == 0 ||
        (direction != PARKING_ENTRY && direction != PARKING_EXIT)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_storage_ready || s_current.pending.stage != PARKING_PENDING_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    const vehicle_registration_t *vehicle = parking_manager_find(uid);
    if (vehicle == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    bool inside = parking_manager_is_inside(uid);
    if ((direction == PARKING_ENTRY && inside) || (direction == PARKING_EXIT && !inside)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (direction == PARKING_ENTRY &&
        (!s_slots_ready || parking_manager_free_count() == 0)) {
        return ESP_ERR_INVALID_STATE;
    }
    s_candidate = s_current;
    memset(&s_candidate.pending, 0, sizeof(s_candidate.pending));
    s_candidate.pending.stage = PARKING_PENDING_AUTHORIZED;
    s_candidate.pending.direction = direction;
    s_candidate.pending.request_id = request_id;
    memcpy(s_candidate.pending.uid, vehicle->uid, PARKING_UID_SIZE);
    memcpy(s_candidate.pending.plate, vehicle->plate, PARKING_PLATE_SIZE);
    return persist_candidate();
}

esp_err_t parking_manager_confirm_passage(void)
{
    if (!s_storage_ready || s_current.pending.stage == PARKING_PENDING_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_current.pending.stage == PARKING_PENDING_PASSED) {
        return ESP_OK;
    }
    int index = stored_index(&s_current, s_current.pending.uid);
    if (index < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    s_candidate = s_current;
    s_candidate.vehicles[index].inside =
        s_current.pending.direction == PARKING_ENTRY ? 1 : 0;
    s_candidate.pending.stage = PARKING_PENDING_PASSED;
    return persist_candidate();
}

esp_err_t parking_manager_finish(bool cancelled)
{
    if (!s_storage_ready ||
        (cancelled && s_current.pending.stage != PARKING_PENDING_AUTHORIZED) ||
        (!cancelled && s_current.pending.stage != PARKING_PENDING_PASSED)) {
        return ESP_ERR_INVALID_STATE;
    }
    s_candidate = s_current;
    memset(&s_candidate.pending, 0, sizeof(s_candidate.pending));
    return persist_candidate();
}

esp_err_t parking_manager_reset_outside(void)
{
    if (!s_storage_ready || s_current.pending.stage != PARKING_PENDING_NONE ||
        !parking_manager_slots_ready() || parking_manager_physical_free() != PARKING_SLOT_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    if (parking_manager_active_count() == 0) {
        return ESP_OK;
    }

    s_candidate = s_current;
    for (uint32_t i = 0; i < s_candidate.count; ++i) {
        s_candidate.vehicles[i].inside = 0;
    }
    return persist_candidate();
}

void parking_manager_dump(void)
{
    ESP_LOGI(TAG, "storage=%s slots=%s active=%u physical_free=%u available=%u",
             s_storage_ready ? "READY" : "LOCKED", s_slots_ready ? "READY" : "SETTLING",
             parking_manager_active_count(), parking_manager_physical_free(),
             parking_manager_free_count());
    for (uint32_t i = 0; i < s_current.count; ++i) {
        ESP_LOGI(TAG, "UID=%s plate=%s %s", s_current.vehicles[i].uid,
                 s_current.vehicles[i].plate, s_current.vehicles[i].inside ? "INSIDE" : "OUTSIDE");
    }
    if (s_current.pending.stage != PARKING_PENDING_NONE) {
        ESP_LOGW(TAG, "Pending stage=%u direction=%s request=%lu UID=%s plate=%s",
                 (unsigned)s_current.pending.stage,
                 s_current.pending.direction == PARKING_ENTRY ? "ENTRY" : "EXIT",
                 (unsigned long)s_current.pending.request_id,
                 s_current.pending.uid, s_current.pending.plate);
    }
}
