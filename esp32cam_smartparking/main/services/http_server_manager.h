#ifndef HTTP_SERVER_MANAGER_H
#define HTTP_SERVER_MANAGER_H

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t http_server_manager_start(void);

esp_err_t http_server_manager_register_uri(
    const httpd_uri_t *uri
);

#endif