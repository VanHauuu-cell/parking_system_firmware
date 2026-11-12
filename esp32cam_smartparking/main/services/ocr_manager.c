#include "ocr_manager.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "http_server_manager.h"
#include "access_event_manager.h"
#include "parking_link_config.h"
#include "uart_driver.h"
#include "uart_protocol.h"

#define PENDING_TIMEOUT_US (65LL * 1000000)

static const char *TAG = "OCR_MANAGER";
static SemaphoreHandle_t pending_lock;
static bool started;
static bool get_registered;
static bool post_registered;
static bool pending;
static uint32_t pending_id;
static int64_t pending_deadline_us;
static uint32_t last_forwarded_id;

// Caller holds pending_lock.
static void expire_pending(void)
{
    if (pending && esp_timer_get_time() >= pending_deadline_us) {
        ESP_LOGW(TAG, "Expired request=%lu", (unsigned long)pending_id);
        pending = false;
    }
}

static void accept_request(uint8_t camera_id, uint32_t request_id)
{
    if (camera_id != PARKING_CAMERA_ID) {
        ESP_LOGW(TAG, "Ignored request for camera=%u", (unsigned)camera_id);
        return;
    }
    xSemaphoreTake(pending_lock, portMAX_DELAY);
    expire_pending();
    if ((pending && pending_id == request_id) || request_id == last_forwarded_id) {
        xSemaphoreGive(pending_lock);
        return;
    }
    pending_id = request_id;
    pending_deadline_us = esp_timer_get_time() + PENDING_TIMEOUT_US;
    pending = true;
    xSemaphoreGive(pending_lock);
    ESP_LOGI(TAG, "[OCR REQUEST] camera=%u request=%lu",
             (unsigned)camera_id, (unsigned long)request_id);
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t buffer[128];
    char frame[UART_MAX_FRAME_SIZE];
    size_t used = 0;
    bool collecting = false;
    while (1) {
        int length = uart_driver_read(buffer, sizeof(buffer), 50);
        for (int i = 0; i < length; ++i) {
            char c = (char)buffer[i];
            if (c == '<') {
                used = 0;
                collecting = true;
            }
            if (!collecting) {
                continue;
            }
            if (used >= sizeof(frame) - 1) {
                used = 0;
                collecting = false;
                ESP_LOGW(TAG, "UART frame too long");
                continue;
            }
            frame[used++] = c;
            if (c == '>') {
                frame[used] = '\0';
                collecting = false;
                used = 0;
                uint8_t camera_id;
                uint32_t request_id;
                if (uart_protocol_parse_capture_request(frame, &camera_id, &request_id)) {
                    accept_request(camera_id, request_id);
                } else if (!access_event_manager_handle_frame(frame)) {
                    ESP_LOGW(TAG, "Invalid UART request: %s", frame);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t get_request_handler(httpd_req_t *request)
{
    xSemaphoreTake(pending_lock, portMAX_DELAY);
    expire_pending();
    bool available = pending;
    uint32_t id = pending_id;
    xSemaphoreGive(pending_lock);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (!available) {
        httpd_resp_set_status(request, "204 No Content");
        return httpd_resp_send(request, NULL, 0);
    }
    char body[96];
    snprintf(body, sizeof(body), "{\"camera_id\":%u,\"request_id\":%lu}",
             (unsigned)PARKING_CAMERA_ID, (unsigned long)id);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, body);
}

static bool read_integer(const cJSON *item, uint32_t minimum,
                         uint32_t maximum, uint32_t *value)
{
    if (!cJSON_IsNumber(item)) {
        return false;
    }
    double number = item->valuedouble;
    // This form rejects NaN too; the range check precedes the integer cast.
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

static bool valid_plate(const char *plate)
{
    if (plate == NULL) {
        return false;
    }
    size_t length = strlen(plate);
    if (length == 0 || length >= 20) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        char c = plate[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
            return false;
        }
    }
    return true;
}

static esp_err_t post_result_handler(httpd_req_t *request)
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
    uint32_t camera_id = 0, request_id = 0, confidence = 0;
    const char *status = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(json, "status"));
    const char *plate = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(json, "plate"));
    bool valid = read_integer(cJSON_GetObjectItemCaseSensitive(json, "camera_id"), 1, 255, &camera_id)
        && read_integer(cJSON_GetObjectItemCaseSensitive(json, "request_id"), 1, UINT32_MAX, &request_id)
        && read_integer(cJSON_GetObjectItemCaseSensitive(json, "confidence"), 0, 100, &confidence)
        && camera_id == PARKING_CAMERA_ID && status != NULL && plate != NULL;
    bool success = status != NULL && strcmp(status, "OK") == 0;
    bool failure = status != NULL && (strcmp(status, "NO_PLATE") == 0 ||
        strcmp(status, "OCR_FAILED") == 0 || strcmp(status, "ERROR") == 0);
    int64_t server_time_ms = 0;
    const cJSON *time_item = cJSON_GetObjectItemCaseSensitive(json, "server_time_ms");
    bool time_valid = cJSON_IsNumber(time_item) &&
        time_item->valuedouble >= (double)PARKING_TIMESTAMP_MIN_MS &&
        time_item->valuedouble <= (double)PARKING_TIMESTAMP_MAX_MS;
    if (time_valid) {
        server_time_ms = (int64_t)time_item->valuedouble;
        time_valid = (double)server_time_ms == time_item->valuedouble;
    }
    valid = valid && ((success && valid_plate(plate) && time_valid) ||
                     (failure && plate[0] == '\0' && confidence == 0));
    if (!valid) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid result fields");
    }

    xSemaphoreTake(pending_lock, portMAX_DELAY);
    expire_pending();
    if (request_id == last_forwarded_id) {
        xSemaphoreGive(pending_lock);
        cJSON_Delete(json);
        return httpd_resp_sendstr(request, "OK duplicate");
    }
    if (!pending || request_id != pending_id) {
        xSemaphoreGive(pending_lock);
        cJSON_Delete(json);
        httpd_resp_set_status(request, "409 Conflict");
        return httpd_resp_sendstr(request, "Request is no longer pending");
    }
    // Sending one small buffered UART frame while holding the lock prevents a
    // replacement request from changing the pending ID during forwarding.
    esp_err_t ret = success
        ? uart_protocol_send_plate_result(PARKING_CAMERA_ID, request_id, plate,
                                          (uint8_t)confidence, server_time_ms)
        : uart_protocol_send_plate_error(PARKING_CAMERA_ID, request_id, status);
    if (ret == ESP_OK) {
        last_forwarded_id = request_id;
        pending = false;
    }
    xSemaphoreGive(pending_lock);
    if (ret != ESP_OK) {
        cJSON_Delete(json);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "UART send failed");
    }
    ESP_LOGI(TAG, "[UART FORWARDED] request=%lu status=%s plate=%s",
             (unsigned long)request_id, status, plate);
    cJSON_Delete(json);
    httpd_resp_set_type(request, "text/plain");
    return httpd_resp_sendstr(request, "OK");
}

esp_err_t ocr_manager_start(void)
{
    if (started) {
        return ESP_OK;
    }
    if (pending_lock == NULL) {
        pending_lock = xSemaphoreCreateMutex();
        if (pending_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    esp_err_t access_ret = access_event_manager_start();
    if (access_ret != ESP_OK) {
        return access_ret;
    }
    // This module remains the only consumer of bytes from the UART RX driver.
    const httpd_uri_t get_uri = {
        .uri = "/ocr-request", .method = HTTP_GET,
        .handler = get_request_handler, .user_ctx = NULL
    };
    const httpd_uri_t post_uri = {
        .uri = "/plate-result", .method = HTTP_POST,
        .handler = post_result_handler, .user_ctx = NULL
    };
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
    if (xTaskCreate(uart_rx_task, "ocr_uart_rx", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    started = true;
    ESP_LOGI(TAG, "Ready: camera=%u UART CAPTURE_REQ, GET /ocr-request POST /plate-result",
             (unsigned)PARKING_CAMERA_ID);
    return ESP_OK;
}
