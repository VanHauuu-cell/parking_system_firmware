#include "stream_manager.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"

static const char *TAG = "STREAM_MANAGER";

#define STREAM_CONTENT_TYPE \
    "multipart/x-mixed-replace;boundary=frame"

#define STREAM_BOUNDARY \
    "\r\n--frame\r\n"

#define STREAM_PART \
    "Content-Type: image/jpeg\r\n" \
    "Content-Length: %u\r\n\r\n"

#define STREAM_TASK_STACK_SIZE     8192
#define STREAM_TASK_PRIORITY       5

static void stream_task(
    void *arg
)
{
    httpd_req_t *request =
        (httpd_req_t *)arg;

    char part_buffer[64];

    esp_err_t ret =
        httpd_resp_set_type(
            request,
            STREAM_CONTENT_TYPE
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to set stream content type"
        );

        httpd_req_async_handler_complete(
            request
        );

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(
        TAG,
        "Stream client connected"
    );

    while (true)
    {
        camera_fb_t *frame =
            esp_camera_fb_get();

        if (frame == NULL)
        {
            ESP_LOGE(
                TAG,
                "Camera capture failed"
            );

            break;
        }

        if (frame->format != PIXFORMAT_JPEG)
        {
            ESP_LOGE(
                TAG,
                "Frame is not JPEG"
            );

            esp_camera_fb_return(frame);

            break;
        }

        int part_length =
            snprintf(
                part_buffer,
                sizeof(part_buffer),
                STREAM_PART,
                frame->len
            );

        ret =
            httpd_resp_send_chunk(
                request,
                part_buffer,
                part_length
            );

        if (ret != ESP_OK)
        {
            esp_camera_fb_return(frame);

            ESP_LOGI(
                TAG,
                "Stream client disconnected"
            );

            break;
        }

        ret =
            httpd_resp_send_chunk(
                request,
                (const char *)frame->buf,
                frame->len
            );

        esp_camera_fb_return(frame);

        if (ret != ESP_OK)
        {
            ESP_LOGI(
                TAG,
                "Failed to send frame"
            );

            break;
        }

        ret =
            httpd_resp_send_chunk(
                request,
                STREAM_BOUNDARY,
                strlen(STREAM_BOUNDARY)
            );

        if (ret != ESP_OK)
        {
            ESP_LOGI(
                TAG,
                "Stream client disconnected"
            );

            break;
        }

        vTaskDelay(
            pdMS_TO_TICKS(30)
        );
    }

    httpd_resp_send_chunk(
        request,
        NULL,
        0
    );

    httpd_req_async_handler_complete(
        request
    );

    ESP_LOGI(
        TAG,
        "Stream task finished"
    );

    vTaskDelete(NULL);
}

static esp_err_t stream_handler(
    httpd_req_t *request
)
{
    httpd_req_t *async_request = NULL;

    esp_err_t ret =
        httpd_req_async_handler_begin(
            request,
            &async_request
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to create async request: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    BaseType_t task_ret =
        xTaskCreate(
            stream_task,
            "stream_task",
            STREAM_TASK_STACK_SIZE,
            async_request,
            STREAM_TASK_PRIORITY,
            NULL
        );

    if (task_ret != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create stream task"
        );

        httpd_req_async_handler_complete(
            async_request
        );

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t stream_manager_start(void)
{
    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    esp_err_t ret =
        http_server_manager_register_uri(
            &stream_uri
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to register /stream: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    ESP_LOGI(
        TAG,
        "Registered /stream"
    );

    return ESP_OK;
}

