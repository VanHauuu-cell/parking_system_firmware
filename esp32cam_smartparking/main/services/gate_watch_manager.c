#include "gate_watch_manager.h"

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

static const char *TAG = "GATE_WATCH";
static SemaphoreHandle_t watch_lock;
static bool started;
static bool get_registered;
static bool post_registered;
static bool watch_active;
static uint32_t watch_id;
static uint32_t last_sequence;

static bool parse_command(const char *frame, bool *arm,
                           uint8_t *camera_id, uint32_t *request_id)
{
    unsigned camera = 0;
    unsigned long long id = 0;
    int end = -1;
    int fields = sscanf(frame, "<GATE_WATCH,%3u,%10llu>%n", &camera, &id, &end);
    bool start = true;
    if (fields != 2 || end < 0 || (size_t)end != strlen(frame)) {
        camera = 0;
        id = 0;
        end = -1;
        fields = sscanf(frame, "<GATE_STOP,%3u,%10llu>%n", &camera, &id, &end);
        start = false;
    }
    if (fields != 2 || end < 0 || (size_t)end != strlen(frame) ||
            camera < 1 || camera > 255 || id < 1 || id > UINT32_MAX) {
        return false;
    }
    *arm = start;
    *camera_id = (uint8_t)camera;
    *request_id = (uint32_t)id;
    return true;
}

bool gate_watch_manager_handle_frame(const char *frame)
{
    if (!started || frame == NULL) {
        return false;
    }
    bool arm;
    uint8_t camera_id;
    uint32_t request_id;
    if (!parse_command(frame, &arm, &camera_id, &request_id)) {
        return false;
    }
    if (camera_id != PARKING_CAMERA_ID) {
        ESP_LOGW(TAG, "Ignored gate command for camera=%u", (unsigned)camera_id);
        return true;
    }
    xSemaphoreTake(watch_lock, portMAX_DELAY);
    if (arm) {
        if (request_id != watch_id) {
            watch_id = request_id;
            last_sequence = 0;
            watch_active = true;
            ESP_LOGI(TAG, "ARM request=%lu", (unsigned long)request_id);
        }
        // Same ID is idempotent, including after its matching STOP.
    } else if (watch_active && request_id == watch_id) {
        watch_active = false;
        ESP_LOGI(TAG, "STOP request=%lu", (unsigned long)request_id);
    }
    xSemaphoreGive(watch_lock);
    return true;
}

static bool read_integer(const cJSON *item, uint32_t minimum,
                         uint32_t maximum, uint32_t *value)
{
    if (!cJSON_IsNumber(item)) {
        return false;
    }
    double number = item->valuedouble;
    if (!(number >= (double)minimum && number <= (double)maximum)) {
        return false;
    }
    uint32_t integer = (uint32_t)number;
    if ((double)integer != number) {
        return false;
    }
    *value = integer;
    return true;
}

static esp_err_t get_watch_handler(httpd_req_t *request)
{
    xSemaphoreTake(watch_lock, portMAX_DELAY);
    bool active = watch_active;
    uint32_t id = watch_id;
    uint32_t sequence = last_sequence;
    xSemaphoreGive(watch_lock);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (!active) {
        httpd_resp_set_status(request, "204 No Content");
        return httpd_resp_send(request, NULL, 0);
    }
    char body[128];
    snprintf(body, sizeof(body),
             "{\"camera_id\":%u,\"request_id\":%lu,\"last_sequence\":%lu}",
             (unsigned)PARKING_CAMERA_ID, (unsigned long)id, (unsigned long)sequence);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, body);
}

static esp_err_t post_status_handler(httpd_req_t *request)
{
    char body[256];
    if (request->content_len == 0 || request->content_len >= sizeof(body)) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid body size");
        return ESP_FAIL;
    }
    size_t received = 0;
    while (received < request->content_len) {
        int length = httpd_req_recv(request, body + received,
                                    request->content_len - received);
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
    if (!cJSON_IsObject(json)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    }
    uint32_t camera_id = 0, request_id = 0, sequence = 0;
    const char *state = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(json, "state"));
    bool valid = read_integer(cJSON_GetObjectItemCaseSensitive(json, "camera_id"), 1, 255, &camera_id)
        && read_integer(cJSON_GetObjectItemCaseSensitive(json, "request_id"), 1, UINT32_MAX, &request_id)
        && read_integer(cJSON_GetObjectItemCaseSensitive(json, "sequence"), 1, UINT32_MAX, &sequence)
        && camera_id == PARKING_CAMERA_ID && state != NULL
        && (strcmp(state, "PRESENT") == 0 || strcmp(state, "CLEAR") == 0);
    if (!valid) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid gate status fields");
    }
    xSemaphoreTake(watch_lock, portMAX_DELAY);
    if (!watch_active || request_id != watch_id) {
        xSemaphoreGive(watch_lock);
        cJSON_Delete(json);
        httpd_resp_set_status(request, "409 Conflict");
        return httpd_resp_sendstr(request, "Watch is no longer active");
    }
    if (sequence <= last_sequence) {
        xSemaphoreGive(watch_lock);
        cJSON_Delete(json);
        return httpd_resp_sendstr(request, "OK duplicate or older sequence");
    }
    char frame[UART_MAX_FRAME_SIZE];
    int length = snprintf(frame, sizeof(frame), "<GATE_STATUS,%u,%lu,%lu,%s>",
                          (unsigned)PARKING_CAMERA_ID, (unsigned long)request_id,
                          (unsigned long)sequence, state);
    esp_err_t ret = length > 0 && (size_t)length < sizeof(frame)
        ? uart_driver_write((const uint8_t *)frame, (size_t)length)
        : ESP_ERR_INVALID_SIZE;
    if (ret == ESP_OK) {
        last_sequence = sequence;
    }
    xSemaphoreGive(watch_lock);
    if (ret != ESP_OK) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "UART send failed");
    }
    ESP_LOGI(TAG, "FORWARDED request=%lu sequence=%lu state=%s",
             (unsigned long)request_id, (unsigned long)sequence, state);
    cJSON_Delete(json);
    return httpd_resp_sendstr(request, "OK");
}

esp_err_t gate_watch_manager_start(void)
{
    if (started) {
        return ESP_OK;
    }
    if (watch_lock == NULL) {
        watch_lock = xSemaphoreCreateMutex();
        if (watch_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    const httpd_uri_t get_uri = {
        .uri = "/gate-watch", .method = HTTP_GET,
        .handler = get_watch_handler, .user_ctx = NULL
    };
    const httpd_uri_t post_uri = {
        .uri = "/gate-status", .method = HTTP_POST,
        .handler = post_status_handler, .user_ctx = NULL
    };
    // Keep registration flags so a startup retry cannot register duplicates.
    if (!get_registered) {
        esp_err_t ret = http_server_manager_register_uri(&get_uri);
        if (ret != ESP_OK) {
            return ret;
        }
        get_registered = true;
    }
    if (!post_registered) {
        esp_err_t ret = http_server_manager_register_uri(&post_uri);
        if (ret != ESP_OK) {
            return ret;
        }
        post_registered = true;
    }
    started = true;
    ESP_LOGI(TAG, "Ready: camera=%u GET /gate-watch POST /gate-status",
             (unsigned)PARKING_CAMERA_ID);
    return ESP_OK;
}
