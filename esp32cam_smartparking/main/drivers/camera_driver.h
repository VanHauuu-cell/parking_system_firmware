#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include "esp_err.h"
#include "esp_camera.h"

esp_err_t camera_driver_init(void);

esp_err_t camera_driver_capture(
    camera_fb_t **frame
);

void camera_driver_release(
    camera_fb_t *frame
);

#endif