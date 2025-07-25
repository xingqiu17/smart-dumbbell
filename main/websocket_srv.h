#ifndef _WEBSOCKET_SRV_H_
#define _WEBSOCKET_SRV_H_


#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string>
#include <algorithm>
#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t sendToClient(const char *json);


#endif