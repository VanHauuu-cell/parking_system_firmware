#include "camera_driver.h"

#include "esp_log.h"

#include "camera_config.h"

static const char *TAG = "CAMERA_DRIVER";

esp_err_t camera_driver_init(void)
{
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,

        .pin_d0 = CAMERA_PIN_D0,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d7 = CAMERA_PIN_D7,

        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_pclk = CAMERA_PIN_PCLK,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,

        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,

        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,

        .xclk_freq_hz = 20000000,

        .pixel_format = PIXFORMAT_JPEG,

        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,

        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_LATEST,
        .fb_location = CAMERA_FB_IN_PSRAM
    };

    esp_err_t ret = esp_camera_init(&config);

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Camera initialization failed: 0x%x",
            ret
        );

        return ret;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    sensor->set_hmirror(sensor, 1);

    ESP_LOGI(TAG, "Camera initialized successfully");

    return ESP_OK;
}

esp_err_t camera_driver_capture(camera_fb_t **frame)
{
    if (frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *frame = esp_camera_fb_get();

    if (*frame == NULL)
    {
        ESP_LOGE(TAG, "Camera capture failed");

            return ESP_FAIL;
    }

    return ESP_OK;
}

void camera_driver_release(camera_fb_t *frame)
{
    if (frame != NULL)
    {
        esp_camera_fb_return(frame);
    }
}