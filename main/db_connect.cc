#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "db_connect.h"
#include <string>

static const char *TAG = "UserClient";


static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_HEADER: {
        // 可选：打印响应头，确认是否 chunked
        ESP_LOGD(TAG, "HEADER %s: %s", evt->header_key, evt->header_value);
        break;
    }
    case HTTP_EVENT_ON_DATA: {
        // 把分块数据拼起来
        if (evt->user_data) {
            auto *body = static_cast<std::string*>(evt->user_data);
            body->append(reinterpret_cast<const char*>(evt->data), evt->data_len);
            ESP_LOGD(TAG, "ON_DATA len=%d, total=%d", evt->data_len, (int)body->size());
        }
        break;
    }
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    default:
        break;
    }
    return ESP_OK;
}


// 执行 HTTP GET 并返回响应体字符串
esp_err_t http_get(const std::string& url, std::string& out_body) {
    ESP_LOGI(TAG, "http_get() URL: %s", url.c_str());

    esp_http_client_config_t config = {};
    config.url          = url.c_str();
    config.method       = HTTP_METHOD_GET;
    config.timeout_ms   = 5000;
    config.event_handler= _http_event_handler;     // ★ 注册事件回调
    config.user_data    = &out_body;               // ★ 让回调能把数据塞进来

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    ESP_LOGI(TAG, "http_get() perform result: %s", esp_err_to_name(err));
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "http_get() HTTP status code = %d", status);
        ESP_LOGI(TAG, "http_get() is_chunked = %d", esp_http_client_is_chunked_response(client));
        if (status != 200) {
            ESP_LOGE(TAG, "HTTP GET failed with status %d", status);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP GET perform failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "http_get() raw body (len=%d): %s", (int)out_body.size(), out_body.c_str());

    esp_http_client_cleanup(client);
    return err;
}








// 将 JSON 字符串解析为 User 结构
bool parse_user_json(const std::string& json, User& user) {
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) {
        ESP_LOGE(TAG, "cJSON_Parse failed ");
        ESP_LOGE(TAG, "Bad JSON was: %s", json.c_str());
        return false;
    }
    cJSON *item = nullptr;
    // 解析各字段
    if ((item = cJSON_GetObjectItem(root, "userId"))      ) user.userId   = item->valueint;
    if ((item = cJSON_GetObjectItem(root, "account"))     ) user.account  = item->valuestring;
    if ((item = cJSON_GetObjectItem(root, "name"))        ) user.name     = item->valuestring;
    if ((item = cJSON_GetObjectItem(root, "gender"))      ) user.gender   = item->valueint;
    if ((item = cJSON_GetObjectItem(root, "birthday"))    ) user.birthday = item->valuestring;
    if ((item = cJSON_GetObjectItem(root, "height"))      ) user.height   = static_cast<float>(item->valuedouble);
    if ((item = cJSON_GetObjectItem(root, "weight"))      ) user.weight   = static_cast<float>(item->valuedouble);
    if ((item = cJSON_GetObjectItem(root, "aim"))         ) user.aim      = item->valueint;
    if ((item = cJSON_GetObjectItem(root, "hwWeight"))    ) user.hwWeight = static_cast<float>(item->valuedouble);
    cJSON_Delete(root);
    return true;
}

// 对外接口：根据用户 ID 获取 User 结构
bool getUserInfo(int userId, User& out_user) {
    char url_buf[128];
    snprintf(url_buf, sizeof(url_buf),
             "http://154.9.24.233:8080/api/v1/users/%d",
             userId);
    std::string body;
    esp_err_t err = http_get(url_buf, body);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to fetch user info");
        return false;
    }
    ESP_LOGI(TAG, "getUserInfo raw response (len=%d): %s", int(body.length()), body.c_str());
    if (!parse_user_json(body, out_user)) {
        ESP_LOGE(TAG, "Failed to parse user JSON");
        return false;
    }
    ESP_LOGI(TAG, "User #%d: %s (aim=%d, height=%.1f, weight=%.1f)",
             out_user.userId,
             out_user.name.c_str(),
             out_user.aim,
             out_user.height,
             out_user.weight);
    return true;
}



