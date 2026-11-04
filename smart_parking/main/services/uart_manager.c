#include "uart_manager.h"

#include <string.h>

#include "esp_log.h"
#include "fsm_config.h"
#include "uart_driver.h"

#define RX_CHUNK_SIZE 256
#define POLL_BYTES_PER_LINK 128

typedef struct {
    uart_frame_parser_t parser;
    uint8_t buffer[RX_CHUNK_SIZE];
    size_t used;
    size_t index;
    bool read_warning_shown;
    bool mismatch_warning_shown;
} link_state_t;

static const char *TAG = "UART_MANAGER";
static bool initialized;
static link_state_t links[UART_LINK_COUNT];
static unsigned next_link;
static const unsigned camera_ids[UART_LINK_COUNT] = {
    FSM_ENTRY_CAMERA_ID, FSM_EXIT_CAMERA_ID
};

static bool link_for_camera(uint8_t camera_id, uart_link_t *link)
{
    for (unsigned index = 0; index < UART_LINK_COUNT; ++index) {
        if (camera_id == camera_ids[index]) {
            *link = (uart_link_t)index;
            return true;
        }
    }
    return false;
}

esp_err_t uart_manager_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    if (camera_ids[UART_LINK_ENTRY] == 0 || camera_ids[UART_LINK_ENTRY] > UINT8_MAX ||
            camera_ids[UART_LINK_EXIT] == 0 || camera_ids[UART_LINK_EXIT] > UINT8_MAX ||
            camera_ids[UART_LINK_ENTRY] == camera_ids[UART_LINK_EXIT]) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = uart_driver_init();
    if (ret != ESP_OK) {
        return ret;
    }
    memset(links, 0, sizeof(links));
    next_link = UART_LINK_ENTRY;
    initialized = true;
    return ESP_OK;
}

typedef int (*frame_builder_t)(char *, size_t, uint8_t, uint32_t);

static esp_err_t send_command(uint8_t camera_id, uint32_t request_id,
                              frame_builder_t builder)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uart_link_t link;
    if (!link_for_camera(camera_id, &link)) {
        return ESP_ERR_INVALID_ARG;
    }
    char frame[PARKING_UART_FRAME_SIZE];
    int length = builder(frame, sizeof(frame), camera_id, request_id);
    if (length <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // Never flush RX here: request IDs let the app reject stale responses.
    esp_err_t ret = uart_driver_write_link(link, (const uint8_t *)frame, (size_t)length);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TX %s %s", link == UART_LINK_ENTRY ? "ENTRY" : "EXIT", frame);
    } else {
        ESP_LOGE(TAG, "UART command send failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t uart_manager_request(uint8_t camera_id, uint32_t request_id)
{
    return send_command(camera_id, request_id, uart_protocol_build_capture_request);
}

esp_err_t uart_manager_watch_start(uint8_t camera_id, uint32_t request_id)
{
    return send_command(camera_id, request_id, uart_protocol_build_watch_start);
}

esp_err_t uart_manager_watch_stop(uint8_t camera_id, uint32_t request_id)
{
    return send_command(camera_id, request_id, uart_protocol_build_watch_stop);
}

static bool poll_link(uart_link_t link, uart_message_t *message)
{
    link_state_t *state = &links[link];
    size_t processed = 0;
    while (processed < POLL_BYTES_PER_LINK) {
        if (state->index == state->used) {
            int length = uart_driver_read_link(link, state->buffer, sizeof(state->buffer), 0);
            state->index = 0;
            state->used = 0;
            if (length < 0) {
                if (!state->read_warning_shown) {
                    ESP_LOGW(TAG, "%s UART read failed",
                             link == UART_LINK_ENTRY ? "ENTRY" : "EXIT");
                    state->read_warning_shown = true;
                }
                return false;
            }
            state->read_warning_shown = false;
            if (length == 0) {
                return false;
            }
            state->used = (size_t)length;
        }
        uint8_t byte = state->buffer[state->index++];
        ++processed;
        if (uart_protocol_feed_message(&state->parser, byte, message)) {
            uint8_t camera_id = message->type == UART_MESSAGE_OCR
                ? message->data.ocr.camera_id : message->data.gate.camera_id;
            if (camera_id != camera_ids[link]) {
                if (!state->mismatch_warning_shown) {
                    ESP_LOGW(TAG, "%s UART ignored camera=%u; expected camera=%u",
                             link == UART_LINK_ENTRY ? "ENTRY" : "EXIT",
                             (unsigned)camera_id, camera_ids[link]);
                    state->mismatch_warning_shown = true;
                }
                continue;
            }
            state->mismatch_warning_shown = false;
            if (message->type == UART_MESSAGE_OCR) {
                const uart_ocr_result_t *result = &message->data.ocr;
                ESP_LOGI(TAG, "RX camera=%u request=%lu %s=%s confidence=%u",
                         (unsigned)result->camera_id, (unsigned long)result->request_id,
                         result->success ? "plate" : "error",
                         result->success ? result->plate : result->reason,
                         (unsigned)result->confidence);
            } else {
                const uart_gate_status_t *gate = &message->data.gate;
                ESP_LOGD(TAG, "RX gate camera=%u request=%lu seq=%lu %s",
                         (unsigned)gate->camera_id, (unsigned long)gate->request_id,
                         (unsigned long)gate->sequence, gate->present ? "PRESENT" : "CLEAR");
            }
            // Keep unread bytes in this chunk for the next poll call.
            return true;
        }
    }
    return false;
}

bool uart_manager_poll_message(uart_message_t *message)
{
    if (!initialized || message == NULL) {
        return false;
    }
    unsigned first = next_link;
    for (unsigned offset = 0; offset < UART_LINK_COUNT; ++offset) {
        uart_link_t link = (uart_link_t)((first + offset) % UART_LINK_COUNT);
        if (poll_link(link, message)) {
            // Even a busy ENTRY UART cannot indefinitely starve EXIT replies.
            next_link = ((unsigned)link + 1) % UART_LINK_COUNT;
            return true;
        }
    }
    next_link = (first + 1) % UART_LINK_COUNT;
    return false;
}

bool uart_manager_poll(uart_ocr_result_t *result)
{
    if (result == NULL) {
        return false;
    }
    // Legacy callers do not consume gate events. Keep the wrapper bounded even
    // when the camera is publishing gate heartbeats continuously.
    for (unsigned attempt = 0; attempt < 4; ++attempt) {
        uart_message_t message;
        if (!uart_manager_poll_message(&message)) {
            return false;
        }
        if (message.type == UART_MESSAGE_OCR) {
            *result = message.data.ocr;
            return true;
        }
    }
    return false;
}
