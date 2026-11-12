#include "approval_log.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "fsm_config.h"
#include "nvs.h"

#define APPROVAL_LOG_MAGIC 0x41504C31UL
#define APPROVAL_LOG_VERSION 1UL
#define APPROVAL_LOG_CAPACITY 8U

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    approval_event_t events[APPROVAL_LOG_CAPACITY];
    approval_event_t last_grant[2];
    uint8_t cancelled[2];
} approval_blob_t;

static const char *TAG = "APPROVAL_LOG";
static nvs_handle_t handle;
static bool initialized;
static bool ready;
static approval_blob_t current;
static approval_blob_t candidate;

static int camera_index(uint8_t camera_id)
{
    if (camera_id == FSM_ENTRY_CAMERA_ID) { return 0; }
    if (camera_id == FSM_EXIT_CAMERA_ID) { return 1; }
    return -1;
}

static bool valid_text(const char *text, size_t capacity, size_t exact_length, bool hex_only)
{
    const char *end = memchr(text, '\0', capacity);
    if (end == NULL || end == text ||
        (exact_length > 0 && (size_t)(end - text) != exact_length)) {
        return false;
    }
    for (const char *cursor = text; cursor < end; ++cursor) {
        bool digit = *cursor >= '0' && *cursor <= '9';
        bool letter = *cursor >= 'A' && *cursor <= (hex_only ? 'F' : 'Z');
        if (!digit && !letter) { return false; }
    }
    return true;
}

static bool valid_event(const approval_event_t *event)
{
    return camera_index(event->camera_id) >= 0 && event->request_id != 0 &&
           valid_text(event->event_id, sizeof(event->event_id), 16, true) &&
           valid_text(event->approval_id, sizeof(event->approval_id), 16, true) &&
           valid_text(event->uid, sizeof(event->uid), 8, true) &&
           valid_text(event->plate, sizeof(event->plate), 0, false) &&
           event->approved_at_ms >= APPROVAL_TIME_MIN_MS &&
           event->approved_at_ms <= APPROVAL_TIME_MAX_MS &&
           ((event->kind == 'A' && strcmp(event->event_id, event->approval_id) == 0) ||
            (event->kind == 'C' && strcmp(event->event_id, event->approval_id) != 0));
}

static bool valid_blob(const approval_blob_t *blob)
{
    if (blob->magic != APPROVAL_LOG_MAGIC || blob->version != APPROVAL_LOG_VERSION ||
        blob->count > APPROVAL_LOG_CAPACITY) {
        return false;
    }
    for (uint32_t i = 0; i < blob->count; ++i) {
        if (!valid_event(&blob->events[i])) { return false; }
    }
    for (unsigned i = 0; i < 2; ++i) {
        const approval_event_t *last = &blob->last_grant[i];
        if (blob->cancelled[i] > 1) { return false; }
        if (last->camera_id == 0) {
            if (blob->cancelled[i]) { return false; }
        } else if (!valid_event(last) || last->kind != 'A' ||
                   camera_index(last->camera_id) != (int)i) {
            return false;
        }
    }
    return true;
}

static esp_err_t persist(void)
{
    if (!ready || !valid_blob(&candidate)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = nvs_set_blob(handle, "outbox", &candidate, sizeof(candidate));
    if (ret == ESP_OK) { ret = nvs_commit(handle); }
    if (ret != ESP_OK) {
        ready = false;
        ESP_LOGE(TAG, "Outbox storage failed: %s; restart after checking NVS",
                 esp_err_to_name(ret));
        return ret;
    }
    current = candidate;
    return ESP_OK;
}

esp_err_t approval_log_init(void)
{
    if (initialized) { return ESP_ERR_INVALID_STATE; }
    initialized = true;
    esp_err_t ret = nvs_open("approval_log", NVS_READWRITE, &handle);
    if (ret != ESP_OK) { return ret; }
    size_t length = 0;
    ret = nvs_get_blob(handle, "outbox", NULL, &length);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        memset(&candidate, 0, sizeof(candidate));
        candidate.magic = APPROVAL_LOG_MAGIC;
        candidate.version = APPROVAL_LOG_VERSION;
        ready = true;
        ret = persist();
    } else if (ret == ESP_OK) {
        if (length != sizeof(current)) { return ESP_ERR_INVALID_SIZE; }
        ret = nvs_get_blob(handle, "outbox", &current, &length);
        if (ret == ESP_OK && !valid_blob(&current)) { return ESP_ERR_INVALID_STATE; }
        ready = ret == ESP_OK;
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Outbox ready: %u pending events", (unsigned)current.count);
    }
    return ret;
}

bool approval_log_ready(void)
{
    return ready;
}

bool approval_log_can_grant(void)
{
    return ready && current.count <= APPROVAL_LOG_CAPACITY - 2;
}

unsigned approval_log_pending_count(void)
{
    return (unsigned)current.count;
}

bool approval_log_was_cancelled(uint8_t camera_id, uint32_t request_id, const char *uid)
{
    int index = camera_index(camera_id);
    return ready && index >= 0 && uid != NULL && current.cancelled[index] &&
           current.last_grant[index].camera_id == camera_id &&
           current.last_grant[index].request_id == request_id &&
           strcmp(current.last_grant[index].uid, uid) == 0;
}

static void new_event_id(char id[APPROVAL_EVENT_ID_SIZE])
{
    uint32_t high;
    uint32_t low;
    do {
        high = esp_random();
        low = esp_random();
    } while (high == 0 && low == 0);
    snprintf(id, APPROVAL_EVENT_ID_SIZE, "%08lX%08lX",
             (unsigned long)high, (unsigned long)low);
}

esp_err_t approval_log_record_grant(uint8_t camera_id, uint32_t request_id,
                                  const char *uid, const char *plate,
                                  int64_t approved_at_ms)
{
    int index = camera_index(camera_id);
    if (!ready || index < 0 || uid == NULL || plate == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (strlen(uid) != PARKING_UID_SIZE - 1 || strlen(plate) == 0 ||
        strlen(plate) >= PARKING_PLATE_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    const approval_event_t *last = &current.last_grant[index];
    if (last->camera_id == camera_id && last->request_id == request_id &&
        strcmp(last->uid, uid) == 0 && strcmp(last->plate, plate) == 0 &&
        last->approved_at_ms == approved_at_ms) {
        return ESP_OK;
    }
    if (!approval_log_can_grant()) { return ESP_ERR_NO_MEM; }
    candidate = current;
    approval_event_t *event = &candidate.events[candidate.count];
    memset(event, 0, sizeof(*event));
    event->camera_id = camera_id;
    event->request_id = request_id;
    event->kind = 'A';
    event->approved_at_ms = approved_at_ms;
    snprintf(event->uid, sizeof(event->uid), "%s", uid);
    snprintf(event->plate, sizeof(event->plate), "%s", plate);
    new_event_id(event->event_id);
    memcpy(event->approval_id, event->event_id, sizeof(event->approval_id));
    if (!valid_event(event)) { return ESP_ERR_INVALID_ARG; }
    candidate.last_grant[index] = *event;
    candidate.cancelled[index] = 0;
    ++candidate.count;
    return persist();
}

esp_err_t approval_log_record_cancel(uint8_t camera_id, uint32_t request_id,
                                   const char *uid)
{
    int index = camera_index(camera_id);
    if (!ready || index < 0 || uid == NULL) { return ESP_ERR_INVALID_STATE; }
    const approval_event_t *last = &current.last_grant[index];
    if (last->camera_id != camera_id || last->request_id != request_id ||
        strcmp(last->uid, uid) != 0) {
        // Pending records created by the previous firmware have no approval event.
        ESP_LOGW(TAG, "Cancellation has no stored approval for camera=%u request=%lu",
                 (unsigned)camera_id, (unsigned long)request_id);
        return ESP_OK;
    }
    if (current.cancelled[index]) { return ESP_OK; }
    if (current.count >= APPROVAL_LOG_CAPACITY) { return ESP_ERR_NO_MEM; }
    candidate = current;
    approval_event_t *event = &candidate.events[candidate.count];
    *event = *last;
    event->kind = 'C';
    do { new_event_id(event->event_id); }
    while (strcmp(event->event_id, event->approval_id) == 0);
    ++candidate.count;
    candidate.cancelled[index] = 1;
    return persist();
}

bool approval_log_peek(approval_event_t *event)
{
    if (!ready || current.count == 0 || event == NULL) { return false; }
    *event = current.events[0];
    return true;
}

esp_err_t approval_log_ack(uint8_t camera_id, const char *event_id)
{
    if (!ready) { return ESP_ERR_INVALID_STATE; }
    if (event_id == NULL || current.count == 0 ||
        current.events[0].camera_id != camera_id ||
        strcmp(current.events[0].event_id, event_id) != 0) {
        return ESP_OK;
    }
    candidate = current;
    --candidate.count;
    memmove(candidate.events, candidate.events + 1,
            candidate.count * sizeof(candidate.events[0]));
    memset(&candidate.events[candidate.count], 0, sizeof(candidate.events[0]));
    esp_err_t ret = persist();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Stored acknowledgement: %s; %u events remain", event_id,
                 (unsigned)current.count);
    }
    return ret;
}
