#include "http_server_manager.h"

#include "esp_log.h"

static const char *TAG = "HTTP_SERVER";

static httpd_handle_t server = NULL;

esp_err_t http_server_manager_start(void)
{
    if (server != NULL)
    {
        return ESP_OK;
    }

    httpd_config_t config =
        HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;

    esp_err_t ret = httpd_start(
        &server,
        &config
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to start HTTP server"
        );

        server = NULL;

        return ret;
    }

    ESP_LOGI(
        TAG,
        "HTTP server started on port %d",
        config.server_port
    );

    return ESP_OK;
}

esp_err_t http_server_manager_register_uri(
    const httpd_uri_t *uri
)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (uri == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return httpd_register_uri_handler(
        server,
        uri
    );
}