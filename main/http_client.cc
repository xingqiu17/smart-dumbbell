
#include "http_client.h"

static const char *TAG_HTTP = "HTTP_SRV";

/* ① 线程安全环形队列 —— 保存服务器返回的分数 */
static QueueHandle_t s_score_queue = nullptr;

/* ② 服务器地址（请按需替换）*/
#define SERVER_URL "http://154.9.24.233:8000/score"   // <-- 修改成你的 IP:PORT


static esp_err_t http_evt_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        /* user_data 保存了我们传进来的 std::string* 指针 */
        std::string *p = static_cast<std::string*>(evt->user_data);
        if (p && evt->data_len)
            p->append((char*)evt->data, evt->data_len);
    }
    return ESP_OK;
}

/* ③ sendToServer: 发送 JSON，收到 {"score":xx.xx} 就丢进队列 */
esp_err_t sendToServer(const char *json_body)
{
    if (!json_body) return ESP_ERR_INVALID_ARG;
    if (!s_score_queue)
    s_score_queue = xQueueCreate(/*len*/ 1, sizeof(float));  //  ← 改这里

    std::string response;                      // 用来接收 body

    esp_http_client_config_t cfg{};      // 全部清零
    cfg.url            = SERVER_URL;
    cfg.method         = HTTP_METHOD_POST;
    cfg.timeout_ms     = 5000;
    cfg.buffer_size    = 1024;
    cfg.buffer_size_tx = 1024;
    cfg.event_handler  = http_evt_handler;   // 回调
    cfg.user_data      = &response;          // 传给回调

    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) return ESP_FAIL;

    esp_http_client_set_header(cli, "Content-Type", "application/json");
    esp_http_client_set_post_field(cli, json_body, strlen(json_body));

    esp_err_t err = esp_http_client_perform(cli);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "HTTP POST failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(cli);
        return err;
    }

    int status = esp_http_client_get_status_code(cli);
    ESP_LOGI(TAG_HTTP, "HTTP status = %d", status);
    ESP_LOGI(TAG_HTTP, "Received body (%d bytes): %s",
             (int)response.size(), response.c_str());

    esp_http_client_cleanup(cli);

    if (status != 200 || response.empty())
        return ESP_FAIL;

    /* ---------- 解析 JSON ---------- */
    cJSON *root = cJSON_Parse(response.c_str());
    if (!root) {
        ESP_LOGE(TAG_HTTP, "JSON parse error: %s", cJSON_GetErrorPtr());
        return ESP_ERR_INVALID_RESPONSE;
    }
    cJSON *sc = cJSON_GetObjectItem(root, "score");
    if (!cJSON_IsNumber(sc)) {
        ESP_LOGE(TAG_HTTP, "no numeric 'score' in response");
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    float v = (float)sc->valuedouble;
    xQueueOverwrite(s_score_queue, &v);        // 覆盖队头
    ESP_LOGI(TAG_HTTP, "Score %.2f enqueued", v);

    cJSON_Delete(root);
    return ESP_OK;
}


/* ④ fetchScore: 立即返回队列里有没有分数 */
bool fetchScore(float &score_out)
{
    if (s_score_queue == nullptr) return false;
    return xQueueReceive(s_score_queue, &score_out, 0) == pdTRUE;
}