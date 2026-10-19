#include "capture_manager.h"

#include <string.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "camera_driver.h"
#include "http_server_manager.h"

static const char *TAG = "CAPTURE_MANAGER";

static esp_err_t capture_handler(
    httpd_req_t *request
)
{
    camera_fb_t *frame = NULL;

    ESP_LOGI(
        TAG,
        "Capture request received"
    );

    esp_err_t ret =
        camera_driver_capture(&frame);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Camera capture failed"
        );

        httpd_resp_send_err(
            request,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Camera capture failed"
        );

        return ret;
    }

    if (frame->format != PIXFORMAT_JPEG)
    {
        ESP_LOGE(
            TAG,
            "Captured frame is not JPEG"
        );

        camera_driver_release(frame);

        httpd_resp_send_err(
            request,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Frame is not JPEG"
        );

        return ESP_ERR_INVALID_RESPONSE;
    }

    ret = httpd_resp_set_type(
        request,
        "image/jpeg"
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to set response content type"
        );

        camera_driver_release(frame);

        return ret;
    }

    ESP_LOGI(
        TAG,
        "Sending captured frame: %u bytes",
        (unsigned)frame->len
    );

    ret = httpd_resp_send(
        request,
        (const char *)frame->buf,
        frame->len
    );

    camera_driver_release(frame);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to send captured frame: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    ESP_LOGI(
        TAG,
        "Capture completed"
    );

    return ESP_OK;
}

esp_err_t capture_manager_start(void)
{
    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_POST,
        .handler = capture_handler,
        .user_ctx = NULL
    };

    esp_err_t ret =
        http_server_manager_register_uri(
            &capture_uri
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to register /capture: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    ESP_LOGI(
        TAG,
        "Registered /capture"
    );

    return ESP_OK;
}

