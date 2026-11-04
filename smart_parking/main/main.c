#include "esp_log.h"
#include "app_main.h"

void app_main(void)
{
    if (!parking_app_start()) {
        ESP_LOGE("MAIN", "Cannot start parking application task");
    }
}
