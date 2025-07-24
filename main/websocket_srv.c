#include "esp_event.h"

#include "esp_http_server.h"
#include "esp_log.h"



static const char *TAG = "WS_SRV";
static httpd_handle_t server = NULL;

/* WebSocket 回调 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    /* 收数据 */
    httpd_ws_frame_t frame = { .payload = NULL };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    frame.payload = malloc(frame.len + 1);
    httpd_ws_recv_frame(req, &frame, frame.len);
    frame.payload[frame.len] = 0;
    ESP_LOGI(TAG, "RX: %s", (char*)frame.payload);

    /* 回 echo */
    httpd_ws_frame_t resp = {
        .final  = true,
        .type   = HTTPD_WS_TYPE_TEXT,
        .len = frame.len,
        .payload = frame.payload
    };
    httpd_ws_send_frame(req, &resp);
    free(frame.payload);
    return ESP_OK;
}

/* 启动服务器 */
void websocket_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 8080;                      // ★ 本地端口
    httpd_start(&server, &cfg);

    httpd_uri_t ws_uri = {
    .uri      = "/ws",
    .method   = HTTP_GET,
    .handler  = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws_uri);

    ESP_LOGI(TAG, "WebSocket server started on :%d/ws", cfg.server_port);
}
