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


esp_err_t http_request(esp_http_client_method_t method,
                       const std::string& url,
                       const std::string* body,
                       std::string& out_body,
                       const char* content_type,
                       const std::vector<std::pair<std::string,std::string>>* extra_headers)
{
    ESP_LOGI(TAG, "http_request() %d %s", (int)method, url.c_str());
    out_body.clear();

    esp_http_client_config_t config = {};
    config.url           = url.c_str();
    config.method        = method;
    config.timeout_ms    = 5000;
    config.event_handler = _http_event_handler; // 在 ON_DATA 里追加到 out_body
    config.user_data     = &out_body;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "http_request: init client failed");
        return ESP_FAIL;
    }

    // 额外请求头
    if (extra_headers) {
        for (auto& kv : *extra_headers) {
            esp_http_client_set_header(client, kv.first.c_str(), kv.second.c_str());
        }
    }

    // 设置请求体
    if (body && !body->empty()) {
        if (content_type && *content_type) {
            esp_http_client_set_header(client, "Content-Type", content_type);
        }
        esp_http_client_set_post_field(client, body->c_str(), body->size());
    } else if (content_type && *content_type) {
        // 没有 body 也允许显式 Content-Type（可选）
        esp_http_client_set_header(client, "Content-Type", content_type);
    }

    esp_err_t err = esp_http_client_perform(client);
    ESP_LOGI(TAG, "http_request() perform: %s", esp_err_to_name(err));
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        bool chunked = esp_http_client_is_chunked_response(client);
        ESP_LOGI(TAG, "http_request() status=%d, chunked=%d, resp_len=%d",
                 status, (int)chunked, (int)out_body.size());

        // 统一把 2xx 当作成功（200/201/204…）
        if (status < 200 || status >= 300) {
            ESP_LOGE(TAG, "http_request() unexpected status %d", status);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "http_request() perform failed: %s", esp_err_to_name(err));
    }

    if (!out_body.empty()) {
        ESP_LOGI(TAG, "http_request() body: %.*s", (int)out_body.size(), out_body.c_str());
    } else {
        ESP_LOGI(TAG, "http_request() body: <empty>");
    }

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
    esp_err_t err = http_request(HTTP_METHOD_GET, url_buf, /*body*/nullptr, body);
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


// —— 训练计划：查询某日所有计划 ——
// 返回值：true 表示 HTTP 2xx 且 out_json 有（可能是 "[]"）
// 注意：这里直接把后端 JSON 转给上层，设备侧不做结构化解析
bool getPlansOfDay(int userId, const std::string& date, std::string& out_json) {
    char url[256];
    // date 必须是 YYYY-MM-DD（已在上层保证），无需额外编码
    snprintf(url, sizeof(url),
             "http://154.9.24.233:8080/api/plan/session/day?userId=%d&date=%s",
             userId, date.c_str());

    std::string body;
    esp_err_t err = http_request(HTTP_METHOD_GET, url, /*body*/nullptr, body);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "getPlansOfDay failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "getPlansOfDay OK (len=%d)", (int)body.size());
    out_json.swap(body);
    return true;
}

// —— 训练计划：创建某日计划 ——
// items_json 必须是 JSON 数组字符串（例如：[{"type":1,"number":12,"tOrder":1,"tWeight":10}, ...]）
bool createPlanOfDay(int userId, const std::string& date,
                     const std::string& items_json,
                     std::string& out_json) {
    // 构建请求体：{"userId":..,"date":"YYYY-MM-DD","items":[...]}
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "userId", userId);
    cJSON_AddStringToObject(root, "date", date.c_str());

    cJSON* items = cJSON_Parse(items_json.c_str());
    if (!items || !cJSON_IsArray(items)) {
        ESP_LOGE(TAG, "createPlanOfDay: items_json is not a valid JSON array: %s",
                 items_json.c_str());
        if (items) cJSON_Delete(items);
        cJSON_Delete(root);
        return false;
    }
    // 交给 root 托管
    cJSON_AddItemToObject(root, "items", items);

    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}";
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    std::string body;
    esp_err_t err = http_request(HTTP_METHOD_POST,
                                 "http://154.9.24.233:8080/api/plan/session",
                                 &payload, body, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "createPlanOfDay failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "createPlanOfDay OK (len=%d)", (int)body.size());
    out_json.swap(body);
    return true;
}


// —— 训练记录：创建某日训练记录 ——
// items_json 必须是 JSON 数组字符串（包含 avgScore 及 works 数组）
bool createRecordOfDay(int userId, const std::string& date,
                       const std::string& items_json,
                       std::string& out_json) {
    // 构建请求体：{"userId":..,"date":"YYYY-MM-DD","items":[...]}
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "userId", userId);
    cJSON_AddStringToObject(root, "date", date.c_str());

    cJSON* items = cJSON_Parse(items_json.c_str());
    if (!items || !cJSON_IsArray(items)) {
        ESP_LOGE(TAG, "createRecordOfDay: items_json is not a valid JSON array: %s",
                 items_json.c_str());
        if (items) cJSON_Delete(items);
        cJSON_Delete(root);
        return false;
    }
    cJSON_AddItemToObject(root, "items", items);

    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}"; // 构造请求体
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    std::string body;
    esp_err_t err = http_request(HTTP_METHOD_POST,
                                 "http://154.9.24.233:8080/api/log/session",
                                 &payload, body, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "createRecordOfDay failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "createRecordOfDay OK (len=%d)", (int)body.size());
    out_json.swap(body);
    return true;
}


// —— 训练计划：标记计划完成 ——
// PATCH /api/plan/session/{sessionId}/complete  body: {"complete": true/false}
bool patchPlanComplete(int sessionId, bool complete, std::string& out_json) {
    char url[256];
    snprintf(url, sizeof(url),
             "http://154.9.24.233:8080/api/plan/session/%d/complete",
             sessionId);

    // 组装 JSON：{"complete": true/false}
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "complete", complete ? 1 : 0);
    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}";
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    // 发送 PATCH（若你所用的 ESP-IDF 不支持 PATCH，可参考后面的“兼容写法”）
    std::string body;
    esp_err_t err = http_request(HTTP_METHOD_PATCH, url, &payload, body, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE("UserClient", "patchPlanComplete failed: %s", esp_err_to_name(err));
        return false;
    }

    // 204 No Content 情况下 body 为空，属于正常
    out_json.swap(body);
    ESP_LOGI("UserClient", "patchPlanComplete OK (sessionId=%d, complete=%d)",
             sessionId, (int)complete);
    return true;
}



