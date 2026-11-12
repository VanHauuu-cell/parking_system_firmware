#include "access_event_manager.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "http_server_manager.h"
#include "parking_link_config.h"
#include "uart_driver.h"
#include "uart_protocol.h"

// Includes the terminator. The longest supported frame is 108 characters.
_Static_assert(sizeof("<ACCESS_EVENT,255,4294967295,FFFFFFFFFFFFFFFF,"
                      "FFFFFFFFFFFFFFFF,C,FFFFFFFF,AAAAAAAAAAAAAAAAAAA,4102444800000>")
               <= UART_MAX_FRAME_SIZE, "ACCESS_EVENT exceeds the UART frame buffer");

typedef struct {
    uint8_t camera_id;
    uint32_t request_id;
    char event_id[17];
    char approval_id[17];
    char kind;
    char uid[9];
    char plate[20];
    int64_t approved_at_ms;
} access_event_t;

static const char *TAG = "ACCESS_EVENT";
static SemaphoreHandle_t event_lock;
static bool started;
static bool get_registered;
static bool ack_registered;
static bool pending;
static access_event_t pending_event;
static char last_ack_event_id[17];

static bool valid_event_id(const char *value)
{
    return value != NULL && strlen(value) == 16 &&
        strspn(value, "0123456789ABCDEF") == 16 &&
        strcmp(value, "0000000000000000") != 0;
}

static esp_err_t send_ack(const char *event_id)
{
    char frame[UART_MAX_FRAME_SIZE];
    int length = snprintf(frame, sizeof(frame), "<ACCESS_ACK,%u,%s>",
                          (unsigned)PARKING_CAMERA_ID, event_id);
    if (length <= 0 || (size_t)length >= sizeof(frame)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return uart_driver_write((const uint8_t *)frame, (size_t)length);
}

bool access_event_manager_handle_frame(const char *frame)
{
    if (!started || frame == NULL) {
        return false;
    }
    access_event_t event = {0};
    unsigned int camera = 0;
    unsigned long long request_id = 0;
    unsigned long long approved_at_ms = 0;
    int end = -1;
    int fields = sscanf(frame,
        "<ACCESS_EVENT,%3u,%10llu,%16[0-9A-F],%16[0-9A-F],%c,%8[0-9A-F],%19[A-Z0-9],%13llu>%n",
        &camera, &request_id, event.event_id, event.approval_id, &event.kind,
        event.uid, event.plate, &approved_at_ms, &end);
    if (fields != 8 || end < 0 || (size_t)end != strlen(frame) ||
            camera < 1 || camera > UINT8_MAX || request_id < 1 || request_id > UINT32_MAX ||
            !valid_event_id(event.event_id) || !valid_event_id(event.approval_id) ||
            (event.kind != 'A' && event.kind != 'C') || strlen(event.uid) != 8 ||
            approved_at_ms < (unsigned long long)PARKING_TIMESTAMP_MIN_MS ||
            approved_at_ms > (unsigned long long)PARKING_TIMESTAMP_MAX_MS ||
            (event.kind == 'A' && strcmp(event.event_id, event.approval_id) != 0)) {
        return false;
    }
    if (camera != PARKING_CAMERA_ID) {
        ESP_LOGW(TAG, "Ignored event for camera=%u", camera);
        return true;
    }
    event.camera_id = (uint8_t)camera;
    event.request_id = (uint32_t)request_id;
    event.approved_at_ms = (int64_t)approved_at_ms;

    xSemaphoreTake(event_lock, portMAX_DELAY);
    if (strcmp(last_ack_event_id, event.event_id) == 0) {
        esp_err_t ret = send_ack(event.event_id);
        xSemaphoreGive(event_lock);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Duplicate ACK failed: %s", esp_err_to_name(ret));
        }
        return true;
    }
    if (pending) {
        bool duplicate = strcmp(pending_event.event_id, event.event_id) == 0;
        xSemaphoreGive(event_lock);
        if (!duplicate) {
            ESP_LOGW(TAG, "Pending event retained; MAIN must retry event=%s", event.event_id);
        }
        return true;
    }
    pending_event = event;
    pending = true;
    xSemaphoreGive(event_lock);
    ESP_LOGI(TAG, "Pending event=%s kind=%c request=%lu",
             event.event_id, event.kind, (unsigned long)event.request_id);
    return true;
}

static esp_err_t get_event_handler(httpd_req_t *request)
{
    xSemaphoreTake(event_lock, portMAX_DELAY);
    bool available = pending;
    access_event_t event = pending_event;
    xSemaphoreGive(event_lock);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (!available) {
        httpd_resp_set_status(request, "204 No Content");
        return httpd_resp_send(request, NULL, 0);
    }
    char body[384];
    int length = snprintf(body, sizeof(body),
        "{\"camera_id\":%u,\"request_id\":%lu,\"event_id\":\"%s\","
        "\"approval_id\":\"%s\",\"kind\":\"%s\",\"uid\":\"%s\","
        "\"plate\":\"%s\",\"approved_at_ms\":%" PRId64 "}",
        (unsigned)event.camera_id, (unsigned long)event.request_id,
        event.event_id, event.approval_id,
        event.kind == 'A' ? "APPROVED" : "CANCELLED", event.uid,
        event.plate, event.approved_at_ms);
    if (length <= 0 || (size_t)length >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "Event exceeds response buffer");
    }
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, body, length);
}

static esp_err_t post_ack_handler(httpd_req_t *request)
{
    char body[128];
    if (request->content_len == 0 || request->content_len >= sizeof(body)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid body size");
    }
    size_t received = 0;
    while (received < request->content_len) {
        int length = httpd_req_recv(request, body + received, request->content_len - received);
        if (length <= 0) {
            if (length == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(request);
            }
            return ESP_FAIL;
        }
        received += (size_t)length;
    }
    body[received] = '\0';
    cJSON *json = cJSON_Parse(body);
    const char *event_id = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(json, "event_id"));
    if (!cJSON_IsObject(json) || !valid_event_id(event_id)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid event_id");
    }
    xSemaphoreTake(event_lock, portMAX_DELAY);
    bool matches_pending = pending && strcmp(pending_event.event_id, event_id) == 0;
    if (!matches_pending && strcmp(last_ack_event_id, event_id) != 0) {
        xSemaphoreGive(event_lock);
        cJSON_Delete(json);
        httpd_resp_set_status(request, "409 Conflict");
        return httpd_resp_sendstr(request, "Event is no longer pending");
    }
    // PC commits SQLite before sending this ACK. MAIN retains its NVS record
    // until this UART acknowledgement arrives, covering CAM restarts and loss.
    esp_err_t ret = send_ack(event_id);
    if (ret == ESP_OK && matches_pending) {
        memcpy(last_ack_event_id, event_id, sizeof(last_ack_event_id));
        pending = false;
    }
    xSemaphoreGive(event_lock);
    if (ret != ESP_OK) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "UART ACK failed");
    }
    ESP_LOGI(TAG, "PC committed event=%s; UART ACK queued", event_id);
    cJSON_Delete(json);
    return httpd_resp_sendstr(request, "OK");
}

esp_err_t access_event_manager_start(void)
{
    if (started) {
        return ESP_OK;
    }
    if (event_lock == NULL) {
        event_lock = xSemaphoreCreateMutex();
        if (event_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    const httpd_uri_t get_uri = {
        .uri = "/access-event", .method = HTTP_GET,
        .handler = get_event_handler, .user_ctx = NULL,
    };
    const httpd_uri_t ack_uri = {
        .uri = "/access-event-ack", .method = HTTP_POST,
        .handler = post_ack_handler, .user_ctx = NULL,
    };
    if (!get_registered) {
        esp_err_t ret = http_server_manager_register_uri(&get_uri);
        if (ret != ESP_OK) {
            return ret;
        }
        get_registered = true;
    }
    if (!ack_registered) {
        esp_err_t ret = http_server_manager_register_uri(&ack_uri);
        if (ret != ESP_OK) {
            return ret;
        }
        ack_registered = true;
    }
    started = true;
    ESP_LOGI(TAG, "Ready: camera=%u GET /access-event POST /access-event-ack",
             (unsigned)PARKING_CAMERA_ID);
    return ESP_OK;
}
