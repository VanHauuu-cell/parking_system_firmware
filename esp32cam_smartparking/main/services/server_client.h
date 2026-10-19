#ifndef SERVER_CLIENT_H
#define SERVER_CLIENT_H

#include "esp_err.h"
#include "esp_camera.h"

esp_err_t server_client_init(void);

esp_err_t server_client_upload_image(
    const camera_fb_t *frame
);

#endif