#include "remote_data_service.h"
#include "esp_log.h"
#include "cJSON.h"

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
    return getUserInfo(userId, out_user);   // 复用 db_connect.cc
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
    cJSON_free(raw);
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
