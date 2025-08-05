#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string>
#include <algorithm>
#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "WS_SRV";
static httpd_handle_t server = NULL;
static int s_ws_fd = -1;        // 保存握手完成后的 client fd

// 全局或静态地创建一个 Settings 实例，用于读写 "pairing" namespace


// 简单的 query 参数解析：从 "/ws?token=XYZ" 中提取 token
static std::string parseQueryParam(const std::string &uri, const std::string &key) {
    auto pos = uri.find('?');
    if (pos == std::string::npos) return "";
    std::string qs = uri.substr(pos + 1);
    std::string prefix = key + "=";
    size_t start = qs.find(prefix);
    if (start == std::string::npos) return "";
    start += prefix.size();
    size_t end = qs.find('&', start);
    return qs.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

// 发送 JSON（text）到已保存的 client fd
esp_err_t sendToClient(const char *json) {
    if (s_ws_fd < 0) return ESP_FAIL;
    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.payload = (uint8_t*)json;
    frame.len     = strlen(json);
    frame.type    = HTTPD_WS_TYPE_TEXT;
    frame.final   = true;
    return httpd_ws_send_frame_async(server, s_ws_fd, &frame);
}

/* WebSocket 回调 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    // —— 握手阶段 ——  
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Handshake done, fd=%d", fd);
        s_ws_fd = fd;


        // 1) 从 Application 里读取本地存储的 token
        auto& kv = Application::GetInstance().GetPairingSettings();
        std::string saved = kv.GetString("pair_token", "");

        // 2) 尝试从 URL 解析客户端带来的 token
        std::string incoming = parseQueryParam(req->uri, "token");

        if (incoming == saved && !saved.empty()) {
            // 自动重连：token 验证通过
            ESP_LOGI(TAG, "Token matched — connected");
            sendToClient(R"({"event":"connected"})");
        } else {
            // 无 token 或 验证失败，都回到配对流程
            ESP_LOGI(TAG, "Trigger manual pairing (saved='%s', incoming='%s')",
                     saved.c_str(), incoming.c_str());
            sendToClient(R"({"event":"pair_request","device_id":"SMARTDB-EEFF"})");
        }
        return ESP_OK;
    }

    // —— 数据帧阶段 ——  
    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    frame.payload = (uint8_t*)malloc(frame.len + 1);
    if (!frame.payload) return ESP_ERR_NO_MEM;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        free(frame.payload);
        return ret;
    }
    frame.payload[frame.len] = 0;  // null-terminate
    ESP_LOGI(TAG, "RX: %s", (char*)frame.payload);

    std::string msg((char*)frame.payload);

    // —— 新增分流逻辑 ——  
    if (msg.find("\"event\":\"pair_response\"") != std::string::npos) {
        // … 原有 pair_response 处理 …
    }
    else if (msg.find("\"event\":\"setUser\"") != std::string::npos) {
       // 直接调用 bind 处理函数，或者推入队列，确保 handleStartTrainingJson 能看到它
       Application::GetInstance().handleStartTrainingJson((char*)frame.payload);
   }
    else if (msg.find("\"event\":\"start_training\"") != std::string::npos) {
        // 1) 将 payload 拷贝到 heap（确保任务里 free）
        char *jsonCopy = strdup((char*)frame.payload);
        // 2) 拿到队列句柄，推送
        auto q = Application::GetJsonQueue();
        if (q) {
            if (xQueueSend(q, &jsonCopy, 0) != pdTRUE) {
                ESP_LOGW(TAG, "训练计划队列已满，丢弃消息");
                free(jsonCopy);
            }
        } else {
            free(jsonCopy);
        }
    }
    else if (msg.find("\"event\":\"skip_rest\"") != std::string::npos ||
             msg.find("\"event\":\"exit_training\"") != std::string::npos) {
        Application::GetInstance().handleStartTrainingJson((char*)frame.payload);
    }
    else {
        // —— 保留 echo 逻辑（可选） ——  
        httpd_ws_frame_t resp;
        memset(&resp, 0, sizeof(resp));
        resp.final   = true;
        resp.type    = HTTPD_WS_TYPE_TEXT;
        resp.len     = frame.len;
        resp.payload = frame.payload;
        httpd_ws_send_frame(req, &resp);
    }

    // 6) 释放本地 buffer（jsonCopy 会在消费后 free）
    free(frame.payload);
    return ESP_OK;
}


/* 启动服务器 */
void websocket_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 8080;                      
    httpd_start(&server, &cfg);

    httpd_uri_t ws_uri = {
      .uri         = "/ws",
      .method      = HTTP_GET,
      .handler     = ws_handler,
      .user_ctx    = NULL,
      .is_websocket= true
    };
    httpd_register_uri_handler(server, &ws_uri);

    ESP_LOGI(TAG, "WebSocket server started on :%d/ws", cfg.server_port);
}
