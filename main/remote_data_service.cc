#include "remote_data_service.h"
#include "esp_log.h"
#include "cJSON.h"
#include "db_connect.h"   // 声明了 http_request / getPlansOfDay / createPlanOfDay / User 等

static const char *TAG_RDS = "RemoteDataService";

/* -------------------------------------------------------------- */
/* 单例实例获取                                                   */
/* -------------------------------------------------------------- */
RemoteDataService& RemoteDataService::GetInstance()
{
    static RemoteDataService instance;
    return instance;
}

/* -------------------------------------------------------------- */
/* 用户相关实现                                                   */
/* -------------------------------------------------------------- */
bool RemoteDataService::GetUserInfo(int userId, User& out_user)
{
    return getUserInfo(userId, out_user);   // 复用 user_client/db_connect 里的实现
}

/* ---------- 更新昵称 ---------- */
bool RemoteDataService::UpdateName(int uid, const std::string& name)
{
    char url[160];
    snprintf(url, sizeof(url), "http://154.9.24.233:8080/api/v1/users/%d/name", uid);

    // 构造 JSON
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", name.c_str());
    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}";
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    std::string resp;
    auto err = http_request(HTTP_METHOD_POST, url, &payload, resp, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE(TAG_RDS, "UpdateName uid=%d failed", uid);
        return false;
    }
    ESP_LOGI(TAG_RDS, "UpdateName uid=%d ok", uid);
    return true;
}

/* ---------- 更新身体数据 ---------- */
bool RemoteDataService::UpdateBody(int uid, const std::string& birthday,
                                   float height, float weight, int gender)
{
    char url[160];
    snprintf(url, sizeof(url), "http://154.9.24.233:8080/api/v1/users/%d/body", uid);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "birthday", birthday.c_str()); // "YYYY-MM-DD"
    cJSON_AddNumberToObject(root, "height",   height);
    cJSON_AddNumberToObject(root, "weight",   weight);
    cJSON_AddNumberToObject(root, "gender",   gender);
    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}";
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    std::string resp;
    auto err = http_request(HTTP_METHOD_POST, url, &payload, resp, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE(TAG_RDS, "UpdateBody uid=%d failed", uid);
        return false;
    }
    ESP_LOGI(TAG_RDS, "UpdateBody uid=%d ok", uid);
    return true;
}

/* ---------- 更新训练数据 ---------- */
bool RemoteDataService::UpdateTrainData(int uid, int aim, float hwWeight)
{
    char url[200];
    snprintf(url, sizeof(url), "http://154.9.24.233:8080/api/v1/users/%d/trainData", uid);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "aim",      aim);
    cJSON_AddNumberToObject(root, "hwWeight", hwWeight);
    char* raw = cJSON_PrintUnformatted(root);
    std::string payload = raw ? raw : "{}";
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);

    std::string resp;
    auto err = http_request(HTTP_METHOD_POST, url, &payload, resp, "application/json");
    if (err != ESP_OK) {
        ESP_LOGE(TAG_RDS, "UpdateTrainData uid=%d failed", uid);
        return false;
    }
    ESP_LOGI(TAG_RDS, "UpdateTrainData uid=%d ok", uid);
    return true;
}

/* ---------- 指定 uid 生成 JSON ---------- */
std::string RemoteDataService::GetUserDataJson(int userId)
{
    User u;
    if (!GetUserInfo(userId, u)) {
        ESP_LOGE(TAG_RDS, "GetUserInfo(%d) failed", userId);
        return R"({"success":false,"message":"failed to fetch user"})";
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "userId",   u.userId);
    cJSON_AddStringToObject(root, "name",     u.name.c_str());
    cJSON_AddNumberToObject(root, "gender",   u.gender);
    cJSON_AddStringToObject(root, "birthday", u.birthday.c_str());
    cJSON_AddNumberToObject(root, "height",   u.height);
    cJSON_AddNumberToObject(root, "weight",   u.weight);
    cJSON_AddNumberToObject(root, "hwWeight", u.hwWeight);

    char *raw = cJSON_PrintUnformatted(root);
    std::string json(raw ? raw : "{}");
    if (raw) cJSON_free(raw);
    cJSON_Delete(root);
    return json;
}

/* ---------- 使用当前会话 uid ---------- */
std::string RemoteDataService::GetUserDataJson()
{
    if (current_user_id_ < 0) {
        return R"({"success":false,"message":"user not bound"})";
    }
    return GetUserDataJson(current_user_id_);
}

/* -------------------------------------------------------------- */
/* 训练计划实现                                                   */
/* -------------------------------------------------------------- */

/* === 查询：显式 uid 版本（与头文件声明匹配）=== */
bool RemoteDataService::GetDayPlans(int userId, const std::string& date, std::string& out_json)
{
    out_json.clear();
    std::string json;
    if (!getPlansOfDay(userId, date, json)) {
        ESP_LOGE(TAG_RDS, "GetDayPlans(uid=%d, date=%s) failed", userId, date.c_str());
        return false;
    }
    // 成功：json 一定是数组字符串（无数据则 "[]")
    out_json.swap(json);
    ESP_LOGI(TAG_RDS, "GetDayPlans ok: len=%d", (int)out_json.size());
    return true;
}

bool RemoteDataService::CreateDayPlan(int userId, const std::string& date,
                                      const std::string& items_json,
                                      std::string& out_json)
{
    ESP_LOGI(TAG_RDS, "CreateDayPlan uid=%d date=%s items=%s",
             userId, date.c_str(), items_json.c_str());

    std::string resp;
    if (!createPlanOfDay(userId, date, items_json, resp)) {
        ESP_LOGE(TAG_RDS, "CreateDayPlan(uid=%d) failed", userId);
        return false;
    }
    out_json.swap(resp);  // 返回后端的单条 PlanDayResp（session + items）
    ESP_LOGI(TAG_RDS, "CreateDayPlan ok");
    return true;
}


/* === 训练记录：创建 session + items === */
bool RemoteDataService::CreateDayRecord(int userId, const std::string& date,
                                        const std::string& items_json,
                                        std::string& out_json)
{
    ESP_LOGI(TAG_RDS, "CreateDayRecord uid=%d date=%s items=%s",
             userId, date.c_str(), items_json.c_str());

    std::string resp;
    if (!createRecordOfDay(userId, date, items_json, resp)) {
        ESP_LOGE(TAG_RDS, "CreateDayRecord(uid=%d) failed", userId);
        return false;
    }
    out_json.swap(resp);  // 返回后端的单条 LogDayResp（session + items）
    ESP_LOGI(TAG_RDS, "CreateDayRecord ok");
    return true;
}


bool RemoteDataService::MarkPlanCompleteById(int sessionId)
{
    std::string resp;
    bool ok = patchPlanComplete(sessionId, /*complete=*/true, resp);
    if (!ok) {
        ESP_LOGE(TAG_RDS, "MarkPlanCompleteById failed (sessionId=%d)", sessionId);
        return false;
    }
    ESP_LOGI(TAG_RDS, "MarkPlanCompleteById OK (sessionId=%d)", sessionId);
    return true;
}
