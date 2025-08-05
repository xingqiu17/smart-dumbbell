#ifndef _HTTP_CLIENT_H_
#define _HTTP_CLIENT_H_


#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string>
#include <algorithm>
#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_http_client.h"

esp_err_t sendToServer(const char *json);   // 发送 IMU 序列并在内部取回分数
bool      fetchScore(float &score_out);     // 非阻塞地取最新分数（true = 取到）

#endif