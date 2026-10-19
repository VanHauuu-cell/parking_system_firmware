#include "esp_log.h"
#include "nvs_flash.h"

#include "camera_driver.h"
#include "network_manager.h"
#include "server_client.h"
#include "http_server_manager.h"
#include "stream_manager.h"
#include "capture_manager.h"

static const char *TAG = "CAM_MAIN";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ESP_ERROR_CHECK(
            nvs_flash_init()
        );
    }

    ESP_ERROR_CHECK(
        camera_driver_init()
    );

    ESP_ERROR_CHECK(
        network_manager_init()
    );

    ESP_ERROR_CHECK(
        server_client_init()
    );

    ESP_ERROR_CHECK(
        network_manager_connect()
    );

    ESP_ERROR_CHECK(
        http_server_manager_start()
    );

    ESP_ERROR_CHECK(
        stream_manager_start()
    );

    ESP_ERROR_CHECK(
        capture_manager_start()
    );

    ESP_LOGI(
        TAG,
        "ESP32-CAM application started"
    );
}