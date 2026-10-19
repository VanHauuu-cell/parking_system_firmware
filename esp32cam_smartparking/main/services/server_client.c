#include "server_client.h"

#include "esp_http_client.h"
#include "esp_log.h"

#include "server_config.h"

static const char *TAG = "SERVER_CLIENT";



static char server_url[128];

esp_err_t server_client_init(void)
{
    snprintf(
        server_url,
        sizeof(server_url),
        "http://%s:%d%s",
        SERVER_HOST,
        SERVER_PORT,
        SERVER_UPLOAD_PATH
    );

    ESP_LOGI(
        TAG,
        "Server URL: %s",
        server_url
    );

    return ESP_OK;
}

static esp_err_t http_event_handler(
    esp_http_client_event_t *event
)
{
    switch (event->event_id)
    {
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(
                TAG,
                "Response data: %.*s",
                event->data_len,
                (char *)event->data
            );
            break;

        default:
            break;
    }

    return ESP_OK;
}

esp_err_t server_client_upload_image(
    const camera_fb_t *frame
)
{ 
    if (frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->format != PIXFORMAT_JPEG)
    {
        ESP_LOGE(
            TAG,
            "Frame is not JPEG"
        );

        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t config = {
    .url = server_url,
    .method = HTTP_METHOD_POST,
    .timeout_ms = 5000,
    .event_handler = http_event_handler
};

    esp_http_client_handle_t client =
        esp_http_client_init(&config);

    if (client == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize HTTP client"
        );

        return ESP_FAIL;
    }

    esp_http_client_set_header(
        client,
        "Content-Type",
        "image/jpeg"
    );

    esp_http_client_set_post_field(
        client,
        (const char *)frame->buf,
        frame->len
    );

    ESP_LOGI(
        TAG,
        "Uploading image: %u bytes",
        (unsigned)frame->len
    );

    esp_err_t ret =
        esp_http_client_perform(client);


    if (ret == ESP_OK)
    {
        int status =
            esp_http_client_get_status_code(client);

        int content_length =
            esp_http_client_get_content_length(client);

        ESP_LOGI(
            TAG,
            "HTTP status: %d",
            status
        );

        ESP_LOGI(
            TAG,
            "Response length: %d",
            content_length
        );
    }
    else
    {
        ESP_LOGE(
            TAG,
            "HTTP request failed: %s",
            esp_err_to_name(ret)
        );
    }

    esp_http_client_cleanup(client);

    return ret;
}