/*
 * MCP Server Implementation
 * Reference: https://modelcontextprotocol.io/specification/2024-11-05
 */

#include "mcp_server.h"
#include <esp_log.h>
#include <esp_app_desc.h>
#include <algorithm>
#include <cstring>
#include <esp_pthread.h>

#include "application.h"
#include "display.h"
#include "board.h"
#include "bmi2.h"
#include "bmi270.h"
#include "bmm150.h"
#include "remote_data_service.h"
#include "cJSON.h"
#include <ctime>
#include "esp_timer.h"
#include "db_connect.h"
#include "websocket_srv.h"
#include <vector>

#define TAG "MCP"

#define DEFAULT_TOOLCALL_STACK_SIZE 6144



using AppTrainingItem = Application::TrainingItem;


McpServer::McpServer() {
}

McpServer::~McpServer() {
    for (auto tool : tools_) {
        delete tool;
    }
    tools_.clear();
}


bool McpServer::RefreshUser(int uid, const std::string* optional_json) {
    User u{};
    bool ok = false;

    if (optional_json) {               // 优先用现成 JSON，避免额外 GET
        ok = parse_user_json(*optional_json, u);
    }
    if (!ok) {                         // 解析失败或未提供 JSON，则后端拉取
        ok = getUserInfo(uid, u);
    }
    if (!ok) return false;

    {   // 写入内存副本并设定默认训练重量
        std::lock_guard<std::mutex> lk(user_mu_);
        user_current_ = u;
        user_loaded_  = true;
    }

    return true;
}


void McpServer::AddCommonTools() {
    // To speed up the response time, we add the common tools to the beginning of
    // the tools list to utilize the prompt cache.
    // Backup the original tools list and restore it after adding the common tools.
    auto original_tools = std::move(tools_);
    auto& board = Board::GetInstance();



    // 获取用户数据工具
    AddTool("self.user.get_data",
        "When the user asks for user-related data, including name, gender, height, weight, birthday, train aim, and current training weight data,use this tool to reply what user asked. ",
        PropertyList({ Property("userId", kPropertyTypeInteger) }),
        [this](const PropertyList&) -> ReturnValue {
            int uid = this->current_user_id_;
            ESP_LOGI(TAG, "Tool self.user.get_data using stored userId=%d", uid);
            // 你原有：取回 JSON
            std::string js = RemoteDataService::GetInstance().GetUserDataJson(uid);

            // 新增：用已拿到的 JSON 刷新本地 User（解析失败则内部会自己再 getUserInfo）
            RefreshUser(uid, &js);

            return js;  // 对外行为不变
        });

    // 修改昵称
    AddTool("self.user.set_name",
        "Update user's display name.",
        PropertyList({
            Property("name", kPropertyTypeString)   // 1~16 字符
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";
            RefreshUser(uid);
            auto name = props["name"].value<std::string>();
            ESP_LOGI(TAG, "set_name uid=%d, name=%s", uid, name.c_str());
            bool ok = RemoteDataService::GetInstance().UpdateName(uid, name);
            if (ok) { // 写后：本地同步
                std::lock_guard<std::mutex> lk(user_mu_);
                if (user_loaded_) user_current_.name = name;
            }
            return ok ? R"({"success":true})" : R"({"success":false})";
        });

    // 修改身体数据：birthday(YYYY-MM-DD)、height(cm)、weight(kg)、gender(0/1/2)
    // 规则：只传你想修改的字段；
    //  - birthday/height/weight 传空字符串 "" 表示保持当前值（也可以直接省略这个字段）；
    //  - gender 传 -1 表示保持当前值（也可以直接省略这个字段）。
    AddTool("self.user.set_body",
        "Update user's body data. Only include fields you want to change. "
        "If you don't want to change a field: omit it (or use empty string for birthday/height/weight; use -1 for gender). "
        "Fields: birthday(YYYY-MM-DD), height(cm as string), weight(kg as string), gender(0 unknown,1 male,2 female)."
        "weight unit is kg, as usual,you can set the num what you have listened",
        PropertyList({
            // 关键：给字符串字段默认值 ""，给 gender 允许 -1 并以 -1 作为默认值（=保持当前）
            Property("birthday", kPropertyTypeString,  std::string("")),     // "" 或省略 = 保持
            Property("height",   kPropertyTypeString,  std::string("")),     // "" 或省略 = 保持
            Property("weight",   kPropertyTypeString,  std::string("")),     // "" 或省略 = 保持
            Property("gender",   kPropertyTypeInteger, -1, -1, 2)            // -1 或省略 = 保持
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";
            RefreshUser(uid);

            // 1) 基线：先用当前用户数据填满
            User finalVals;
            { std::lock_guard<std::mutex> lk(user_mu_); finalVals = user_current_; }

            bool changed = false;

            // 2) birthday：省略或 "" → 保持；非空 → 覆盖
            try {
                auto s = props["birthday"].value<std::string>();
                if (!s.empty()) { finalVals.birthday = s; changed = true; }
            } catch (...) { /* omitted: keep */ }

            // —— 工具函数：转小写（仅ASCII），便于找 kg/g
            auto toLowerAscii = [](std::string s) {
                for (auto &ch : s) if (ch >= 'A' && ch <= 'Z') ch = char(ch - 'A' + 'a');
                return s;
            };

            // —— 工具函数：解析体重为 kg
            auto parse_weight_kg = [&](const std::string& s) -> std::optional<float> {
                if (s.empty()) return std::nullopt;
                float v = 0.f;
                try { v = std::stof(s); } catch (...) { return std::nullopt; }

                const std::string ls = toLowerAscii(s);
                const bool has_kg  = (ls.find("kg") != std::string::npos) || (s.find("千克") != std::string::npos) || (s.find("公斤") != std::string::npos);
                const bool has_g   = (ls.find("g")  != std::string::npos) || (s.find("克")  != std::string::npos);
                const bool has_jin = (s.find("斤")  != std::string::npos);

                float kg = v;
                if (has_kg) {
                    kg = v;                // 已是 kg
                } else if (has_jin) {
                    kg = v * 0.5f;         // 斤 → kg
                } else if (has_g) {
                    kg = v / 1000.f;       // g  → kg
                } else {
                    kg = v;                // 无单位：默认 kg
                    // 兜底：若数值异常大（常见“把 g 当 kg”），自动按 g→kg 修正
                    if (kg > 400.f && kg < 300000.f) kg = kg / 1000.f;
                }

                // 合理性检查（可按需放宽/收紧）
                if (std::isnan(kg) || kg <= 0.f || kg > 500.f) return std::nullopt;
                ESP_LOGI(TAG, "normalize weight: raw=\"%s\" -> %.2f kg (kg=%d g=%d jin=%d)",
                        s.c_str(), kg, has_kg, has_g, has_jin);
                return kg;
            };

            // —— 工具函数：解析身高为 cm
            auto parse_height_cm = [&](const std::string& s) -> std::optional<float> {
                if (s.empty()) return std::nullopt;
                float v = 0.f;
                try { v = std::stof(s); } catch (...) { return std::nullopt; }

                const std::string ls = toLowerAscii(s);
                const bool has_cm = (ls.find("cm") != std::string::npos) || (s.find("厘米") != std::string::npos);
                const bool has_m  = ((ls.find("m") != std::string::npos) && (ls.find("cm") == std::string::npos)) || (s.find("米") != std::string::npos);

                float cm = v;
                if (has_cm) {
                    cm = v;                // cm
                } else if (has_m) {
                    cm = v * 100.f;        // m → cm
                } else {
                    // 无单位：默认 cm；若数值很小（≤3），按米理解
                    cm = (v <= 3.f ? v * 100.f : v);
                }

                if (std::isnan(cm) || cm <= 0.f || cm > 300.f) return std::nullopt;
                ESP_LOGI(TAG, "normalize height: raw=\"%s\" -> %.2f cm (cm=%d m=%d)",
                        s.c_str(), cm, has_cm, has_m);
                return cm;
            };

            // 3) height：省略/"" → 保持；否则做单位归一化
            try {
                auto s = props["height"].value<std::string>();
                if (!s.empty()) {
                    if (auto h = parse_height_cm(s)) { finalVals.height = *h; changed = true; }
                }
            } catch (...) { /* omitted: keep */ }

            // 4) weight：省略/"" → 保持；否则做单位归一化（kg）
            try {
                auto s = props["weight"].value<std::string>();
                if (!s.empty()) {
                    if (auto w = parse_weight_kg(s)) { finalVals.weight = *w; changed = true; }
                }
            } catch (...) { /* omitted: keep */ }

            // 5) gender：省略或 -1 → 保持；0/1/2 → 覆盖
            try {
                int g = props["gender"].value<int>();
                if (g != -1) { finalVals.gender = g; changed = true; }
            } catch (...) { /* omitted: keep */ }

            if (!changed) {
                ESP_LOGI(TAG, "set_body uid=%d: no fields provided, skip update", uid);
                return R"({"success":true,"message":"noop"})";
            }

            ESP_LOGI(TAG, "set_body uid=%d, birthday=%s, h=%.2f, w=%.2f, g=%d",
                    uid, finalVals.birthday.c_str(), finalVals.height, finalVals.weight, finalVals.gender);

            // 5) 上传“填充后的最终值”
            bool ok = RemoteDataService::GetInstance().UpdateBody(
                uid, finalVals.birthday, finalVals.height, finalVals.weight, finalVals.gender);

            if (ok) {
                std::lock_guard<std::mutex> lk(user_mu_);
                if (user_loaded_) user_current_ = finalVals;
            }
            return ok ? R"({"success":true})" : R"({"success":false})";
        });




    // 修改训练目标/重量：aim、hwWeight(kg)
    // 说明里明确：不想改就“省略该字段”；若传了也可用哨兵（aim=-1 / hwWeight=""）表示保持当前
    AddTool("self.user.set_train_data",
        "Update user's training aim and hwWeight(kg). Only include fields you want to change. "
        "If you don't want to change aim, omit it (or pass -1). "
        "If you don't want to change hwWeight, omit it (or pass empty string). "
        "Aim: 0 none,1 arm,2 shoulder,3 chest,4 back,5 leg."
        "hwWeight unit: kg (can be float).If you listen Chinese 千克,please use 'kg' as unit.",
        PropertyList({
            Property("aim",      kPropertyTypeInteger, /*default*/ -1, /*min*/ -1, /*max*/ 5),
            Property("hwWeight", kPropertyTypeString,  std::string(""))
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";
            RefreshUser(uid);

            // 基线：用当前值填满
            User finalVals;
            { std::lock_guard<std::mutex> lk(user_mu_); finalVals = user_current_; }

            // aim：若参数被省略 → 这里会抛异常，保持默认；若存在且 == -1 → 也保持默认
            try {
                int aim_in = props["aim"].value<int>();
                if (aim_in != -1) finalVals.aim = aim_in;
            } catch (...) { /* omitted: keep default */ }

            // hwWeight：若参数被省略或空串/无效 → 保持默认；否则覆盖
            try {
                auto s = props["hwWeight"].value<std::string>();
                if (!s.empty()) {
                    try {
                        float v = std::stof(s);
                        if (!(std::isnan(v) || v < 0.0f)) finalVals.hwWeight = v;
                    } catch (...) { /* parse failed: keep default */ }
                }
            } catch (...) { /* omitted: keep default */ }

            ESP_LOGI(TAG, "set_train_data uid=%d, aim=%d, hwWeight=%.2f",
                    uid, finalVals.aim, finalVals.hwWeight);

            bool ok = RemoteDataService::GetInstance().UpdateTrainData(uid, finalVals.aim, finalVals.hwWeight);
            if (ok) { std::lock_guard<std::mutex> lk(user_mu_); if (user_loaded_) user_current_ = finalVals; }
            return ok ? R"({"success":true})" : R"({"success":false})";
        });



    

    // 查询某日的所有计划（只读，不创建）
    AddTool("self.plan.get_day",
            "If user wants to confirm the plan of the day,please use this tools."
            "Only Use this tool to confirm the plan of the day,not start or create."
            "Get all plans for the given date of the current user. "
            "Args: date (YYYY-MM-DD or 'today'). "
            "Return: JSON array of plan sessions. "
            "Each session object has a 'complete' flag: "
            "`true` means that entire plan session is finished, "
            "`false` means it is still in progress. "
            "Ignore the `complete` fields inside 'items'.",
        PropertyList({
            Property("date", kPropertyTypeString)
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";

            std::string date = props["date"].value<std::string>();

            // 支持 'today'
            auto normalizeDate = [](std::string& d) {
                if (d.empty() || d == "today" || d == "Today" || d == "TODAY") {
                    time_t now = time(nullptr);
                    struct tm tm_info;
                    localtime_r(&now, &tm_info);
                    char buf[11] = {0}; // "YYYY-MM-DD"
                    strftime(buf, sizeof(buf), "%Y-%m-%d", &tm_info);
                    d.assign(buf);
                }
            };
            normalizeDate(date);

            std::string jsonArr;  // 期望为 "[]" 或 "[{...}]"
            bool ok = RemoteDataService::GetInstance().GetDayPlans(uid, date, jsonArr);
            if (!ok) return R"({"success":false,"message":"fetch_failed"})";

            // 直接把后端返回的数组透传给模型/上层
            return jsonArr;
        });


    // 创建某日计划（包含多个动作）
    AddTool("self.plan.create_day",
        "Create a plan for the given date of the current user. "
        "Args: date (YYYY-MM-DD or 'today'), items (JSON array string). "
        "Each item MUST contain: type(int), number(int), tOrder(int), rest(int). "
        "tWeight(float) is OPTIONAL — include it ONLY if the user explicitly specifies a weight; "
        "`rest` is the pause time *after* this action, in **seconds** (0 means skip rest). "
        "About type: 1=dumbbell-curl."
        "IMPORTANT: type must be an integer ID (do NOT send strings).",
        PropertyList({
            Property("date",  kPropertyTypeString),
            Property("items", kPropertyTypeString)   // JSON 数组字符串
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";

            std::string date  = props["date"].value<std::string>();
            std::string items = props["items"].value<std::string>();

            // 支持 "today"
            auto normalizeDate = [](std::string& d) {
                if (d == "today" || d == "Today" || d == "TODAY" || d.empty()) {
                    time_t now = time(nullptr);
                    struct tm tm_info;
                    localtime_r(&now, &tm_info);
                    char buf[11] = {0}; // "YYYY-MM-DD"
                    strftime(buf, sizeof(buf), "%Y-%m-%d", &tm_info);
                    d.assign(buf);
                }
            };
            normalizeDate(date);

            // 解析 items 原始 JSON
            cJSON* arr = cJSON_Parse(items.c_str());
            if (!arr || !cJSON_IsArray(arr)) {
                if (arr) cJSON_Delete(arr);
                return R"({"success":false,"message":"items_not_array"})";
            }

            // 归一化后的 items（type/number/tOrder 为 int；tWeight 为 float）
            cJSON* norm_items = cJSON_CreateArray();
            const int n = cJSON_GetArraySize(arr);
            for (int i = 0; i < n; ++i) {
                cJSON* it = cJSON_GetArrayItem(arr, i);
                if (!cJSON_IsObject(it)) {
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return R"({"success":false,"message":"items[i]_not_object"})";
                }

                // type：必须是整数
                int type_id = -1;
                if (cJSON* jt = cJSON_GetObjectItemCaseSensitive(it, "type")) {
                    if (cJSON_IsNumber(jt)) type_id = jt->valueint;
                    else { // 明确拒绝字符串等
                        cJSON_Delete(norm_items); cJSON_Delete(arr);
                        return std::string("{\"success\":false,\"message\":\"type_must_be_integer_at_index_")
                            + std::to_string(i+1) + "\"}";
                    }
                }
                if (type_id <= 0) {
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return std::string("{\"success\":false,\"message\":\"bad_type_at_index_")
                        + std::to_string(i+1) + "\"}";
                }

                // number
                int number = -1;
                if (cJSON* jn = cJSON_GetObjectItemCaseSensitive(it, "number")) {
                    if (cJSON_IsNumber(jn)) number = jn->valueint;
                }
                if (number <= 0) {
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return std::string("{\"success\":false,\"message\":\"bad_number_at_index_")
                        + std::to_string(i+1) + "\"}";
                }

                // tOrder
                int tOrder = -1;
                if (cJSON* jo = cJSON_GetObjectItemCaseSensitive(it, "tOrder")) {
                    if (cJSON_IsNumber(jo)) tOrder = jo->valueint;
                }
                if (tOrder <= 0) {
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return std::string("{\"success\":false,\"message\":\"bad_tOrder_at_index_")
                        + std::to_string(i+1) + "\"}";
                }

                float tWeight = this->current_user_tweight_;
                
                ESP_LOGI(TAG, "create_day: default tWeight before parse = %.3f, this=%p",
                        (double)this->current_user_tweight_, this);

                 // 兼容多种命名：tWeight / tweight / weight
                cJSON* jw = cJSON_GetObjectItemCaseSensitive(it, "tWeight");
                if (!jw) jw = cJSON_GetObjectItemCaseSensitive(it, "tweight");
                if (!jw) jw = cJSON_GetObjectItemCaseSensitive(it, "weight");
                if (jw) {
                    if (cJSON_IsNumber(jw)) {
                        tWeight = static_cast<float>(jw->valuedouble);
                    } else if (cJSON_IsString(jw) && jw->valuestring && jw->valuestring[0] != '\0') {
                        try {
                            tWeight = std::stof(jw->valuestring);  // 解析失败则捕获并保留默认
                        } catch (...) {
                            ESP_LOGW(TAG, "create_day: tWeight string parse failed, use default=%.3f",
                                    static_cast<double>(tWeight));
                            // 保持默认，不要改成 -1
                        }
                    } else if (cJSON_IsNull(jw)) {
                        // 明确 null：保留默认
                    } else {
                        // 其它类型：保留默认
                    }
                }

                /* ---- rest ---- */
                int rest = 0;                                           // 默认 0
                if (cJSON* jr = cJSON_GetObjectItemCaseSensitive(it, "rest")) {
                    if (cJSON_IsNumber(jr)) rest = jr->valueint;
                }
                if (rest < 0 || rest > 600) {      // 最大 10 分钟，可按需放宽
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return std::string("{\"success\":false,\"message\":\"bad_rest_at_index_")
                        + std::to_string(i + 1) + "\"}";
                }



                // 最终校验：若默认也无效（<=0），才报错
                if (!(tWeight > 0.0f) || !std::isfinite(tWeight)) {
                    ESP_LOGE(TAG, "create_day: no valid tWeight; default(current_user_tweight_=%.3f) invalid",
                            static_cast<double>(this->current_user_tweight_));
                    cJSON_Delete(norm_items); cJSON_Delete(arr);
                    return std::string("{\"success\":false,\"message\":\"bad_tWeight_at_index_")
                        + std::to_string(i+1) + "\"}";
                }

                cJSON* out = cJSON_CreateObject();
                cJSON_AddNumberToObject(out, "type",    type_id);
                cJSON_AddNumberToObject(out, "number",  number);
                cJSON_AddNumberToObject(out, "tOrder",  tOrder);
                cJSON_AddNumberToObject(out, "tWeight", tWeight); // 以 number 发送
                cJSON_AddNumberToObject(out, "rest", rest);
                cJSON_AddItemToArray(norm_items, out);
            }
            cJSON_Delete(arr);

            // 序列化并发送
            char* norm_str = cJSON_PrintUnformatted(norm_items);
            std::string norm_items_json = norm_str ? norm_str : "[]";
            if (norm_str) cJSON_free(norm_str);
            cJSON_Delete(norm_items);

            ESP_LOGI(TAG, "Tool self.plan.create_day uid=%d date=%s normalized_items=%s",
                    uid, date.c_str(), norm_items_json.c_str());

            std::string resp;
            bool ok = RemoteDataService::GetInstance().CreateDayPlan(uid, date, norm_items_json, resp);
            if (!ok) return R"({"success":false,"message":"create_failed"})";
            return resp;
        });

        AddTool("self.training.start_by_plan",
            "Start a training session by an existing uncompleted plan . "
            "You can confirm the plan by sessionid."
            "Args: sessionId(int, required), date(YYYY-MM-DD or 'today', optional).",
            PropertyList({
                Property("sessionId", kPropertyTypeInteger),
                Property("date",      kPropertyTypeString, std::string("today"))
            }),
            [this](const PropertyList& props) -> ReturnValue {
                int uid = this->current_user_id_;
                if (uid <= 0) return R"({"success":false,"message":"no_user"})";

                const int sid = props["sessionId"].value<int>();
                std::string date = props["date"].value<std::string>();

                // 标准化日期
                auto normalizeDate = [](std::string& d) {
                    if (d.empty() || d == "today" || d == "Today" || d == "TODAY") {
                        time_t now = time(nullptr);
                        struct tm tm_info;
                        localtime_r(&now, &tm_info);
                        char buf[11] = {0}; // "YYYY-MM-DD"
                        strftime(buf, sizeof(buf), "%Y-%m-%d", &tm_info);
                        d.assign(buf);
                    }
                };
                normalizeDate(date);

                // 拉取当天所有计划
                std::string plans_json;
                if (!RemoteDataService::GetInstance().GetDayPlans(uid, date, plans_json)) {
                    return R"({"success":false,"message":"fetch_plans_failed"})";
                }

                // 在数组中找到目标 session
                cJSON* root = cJSON_Parse(plans_json.c_str());
                if (!root || !cJSON_IsArray(root)) {
                    if (root) cJSON_Delete(root);
                    return R"({"success":false,"message":"plans_not_array"})";
                }
                cJSON* found = nullptr;
                int n = cJSON_GetArraySize(root);
                for (int i = 0; i < n; ++i) {
                    cJSON* elem = cJSON_GetArrayItem(root, i);
                    if (!cJSON_IsObject(elem)) continue;
                    cJSON* session = cJSON_GetObjectItemCaseSensitive(elem, "session");
                    if (!session || !cJSON_IsObject(session)) continue;
                    cJSON* jsid = cJSON_GetObjectItemCaseSensitive(session, "sessionId");
                    if (!jsid || !cJSON_IsNumber(jsid)) continue;
                    if (jsid->valueint == sid) { found = elem; break; }
                }
                if (!found) {
                    cJSON_Delete(root);
                    return R"({"success":false,"message":"plan_not_found"})";
                }

                // 提取 items → 转 TrainingItem 向量
                cJSON* items = cJSON_GetObjectItemCaseSensitive(found, "items");
                if (!items || !cJSON_IsArray(items)) {
                    cJSON_Delete(root);
                    return R"({"success":false,"message":"items_not_array"})";
                }

                std::vector<AppTrainingItem> vec; // {type, reps, weight}
                int m = cJSON_GetArraySize(items);
                vec.reserve(m);
                for (int i = 0; i < m; ++i) {
                    cJSON* it = cJSON_GetArrayItem(items, i);
                    if (!cJSON_IsObject(it)) continue;
                    cJSON* jt = cJSON_GetObjectItemCaseSensitive(it, "type");
                    cJSON* jn = cJSON_GetObjectItemCaseSensitive(it, "number");
                    cJSON* jw = cJSON_GetObjectItem(it, "tWeight");
                    cJSON* jr = cJSON_GetObjectItemCaseSensitive(it, "rest");
                    if (!jt || !jn || !cJSON_IsNumber(jt) || !cJSON_IsNumber(jn)) continue;

                    float tWeight = this->current_user_tweight_;
                    if (cJSON_IsNumber(jw)) tWeight = (float)jw->valuedouble;
                    else if (cJSON_IsString(jw) && jw->valuestring) {
                        try { tWeight = std::stof(jw->valuestring); } catch (...) { tWeight = 0.f; }
                    }
                    if (!(tWeight > 0.f)) continue;

                    /* ---- 解析休息秒 ---- */
                    int rest = (jr && cJSON_IsNumber(jr)) ? jr->valueint : 0;
                    if (rest < 0) rest = 0;

                    AppTrainingItem ti{};
                    ti.type   = jt->valueint;
                    ti.reps   = jn->valueint;
                    ti.weight = tWeight;
                    ti.rest   = rest;
                    vec.push_back(ti);
                }
                cJSON_Delete(root);

                if (vec.empty()) {
                    return R"({"success":false,"message":"no_valid_items"})";
                }

                // 发送数据到 app
                cJSON* response = cJSON_CreateObject();
                cJSON_AddStringToObject(response, "event", "training_started");
                cJSON_AddNumberToObject(response, "sessionId", sid);
                cJSON_AddStringToObject(response, "date", date.c_str());  // 使用 .c_str() 转换为 const char*
                cJSON* items_arr = cJSON_CreateArray();
                for (const auto& item : vec) {
                    cJSON* item_obj = cJSON_CreateObject();
                    cJSON_AddNumberToObject(item_obj, "type", item.type);
                    cJSON_AddNumberToObject(item_obj, "reps", item.reps);
                    cJSON_AddNumberToObject(item_obj, "weight", item.weight);
                    cJSON_AddNumberToObject(item_obj, "rest", item.rest);
                    cJSON_AddItemToArray(items_arr, item_obj);
                }
                cJSON_AddItemToObject(response, "items", items_arr);
                sendToClient(cJSON_PrintUnformatted(response));
                cJSON_Delete(response);

                // 切到应用线程启动训练
                Application::GetInstance().Schedule([sid, v = std::move(vec)]() {
                    Application::GetInstance().StartTrainingFromItems(sid, v);
                });

                return R"({"success":true})";
            });


        // ============================================================
        // 2) 跳过休息：直接跳过当前休息，开始下一个动作
        // ============================================================

       AddTool("self.training.skip_rest",
        "Skip the current rest and start the next set immediately."
        "Use this tool when the user wants to skip the rest period and continue with the next set.",
        PropertyList(),   // 无参
        [this](const PropertyList&) -> ReturnValue {
            Application::GetInstance().Schedule([]() {
                Application::GetInstance().SkipRest();
            });
            
            // 发送跳过休息的通知
            cJSON* response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "event", "rest_skipped");
            sendToClient(cJSON_PrintUnformatted(response));
            cJSON_Delete(response);

            return R"({"success":true})";
        });


        // ============================================================
        // 3) 退出训练：直接结束本次训练
        // ============================================================
        AddTool("self.training.exit",
            "Exit current training immediately (finalize and save)."
            "Use this tool when the user wants to stop the current training session and exit.",
            PropertyList(),   // 无参
            [this](const PropertyList&) -> ReturnValue {
                Application::GetInstance().Schedule([]() {
                    Application::GetInstance().ExitTraining();
                });

                // 发送退出训练的通知
                cJSON* response = cJSON_CreateObject();
                cJSON_AddStringToObject(response, "event", "training_exited");
                sendToClient(cJSON_PrintUnformatted(response));
                cJSON_Delete(response);

                return R"({"success":true})";
            });

    



    //获取状态工具
    AddTool("self.get_device_status",
        "Provides the real-time information of the device, including the current status of the audio speaker, screen, battery, network, etc.\n"
        "Use this tool for: \n"
        "1. Answering questions about current condition (e.g. what is the current volume of the audio speaker?)\n"
        "2. As the first step to control the device (e.g. turn up / down the volume of the audio speaker, etc.)",
        PropertyList(),
        [&board](const PropertyList& properties) -> ReturnValue {
            return board.GetDeviceStatusJson();
        });


    //获取/设置音量工具
    AddTool("self.audio_speaker.set_volume", 
        "Set the volume of the audio speaker. If the current volume is unknown, you must call `self.get_device_status` tool first and then call this tool.",
        PropertyList({
            Property("volume", kPropertyTypeInteger, 0, 100)
        }), 
        [&board](const PropertyList& properties) -> ReturnValue {
            auto codec = board.GetAudioCodec();
            codec->SetOutputVolume(properties["volume"].value<int>());
            return true;
        });
    
    auto backlight = board.GetBacklight();
    if (backlight) {
        AddTool("self.screen.set_brightness",
            "Set the brightness of the screen.",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 100)
            }),
            [backlight](const PropertyList& properties) -> ReturnValue {
                uint8_t brightness = static_cast<uint8_t>(properties["brightness"].value<int>());
                backlight->SetBrightness(brightness, true);
                return true;
            });
    }

    auto display = board.GetDisplay();
    if (display && !display->GetTheme().empty()) {
        AddTool("self.screen.set_theme",
            "Set the theme of the screen. The theme can be `light` or `dark`.",
            PropertyList({
                Property("theme", kPropertyTypeString)
            }),
            [display](const PropertyList& properties) -> ReturnValue {
                display->SetTheme(properties["theme"].value<std::string>().c_str());
                return true;
            });
    }

    auto camera = board.GetCamera();
    if (camera) {
        AddTool("self.camera.take_photo",
            "Take a photo and explain it. Use this tool after the user asks you to see something.\n"
            "Args:\n"
            "  `question`: The question that you want to ask about the photo.\n"
            "Return:\n"
            "  A JSON object that provides the photo information.",
            PropertyList({
                Property("question", kPropertyTypeString)
            }),
            [camera](const PropertyList& properties) -> ReturnValue {
                if (!camera->Capture()) {
                    return "{\"success\": false, \"message\": \"Failed to capture photo\"}";
                }
                auto question = properties["question"].value<std::string>();
                return camera->Explain(question);
            });
    }


    

    

    // Restore the original tools list to the end of the tools list
    tools_.insert(tools_.end(), original_tools.begin(), original_tools.end());
}

void McpServer::AddTool(McpTool* tool) {
    // Prevent adding duplicate tools
    if (std::find_if(tools_.begin(), tools_.end(), [tool](const McpTool* t) { return t->name() == tool->name(); }) != tools_.end()) {
        ESP_LOGW(TAG, "Tool %s already added", tool->name().c_str());
        return;
    }

    ESP_LOGI(TAG, "Add tool: %s", tool->name().c_str());
    tools_.push_back(tool);
}

void McpServer::AddTool(const std::string& name, const std::string& description, const PropertyList& properties, std::function<ReturnValue(const PropertyList&)> callback) {
    AddTool(new McpTool(name, description, properties, callback));
}

void McpServer::ParseMessage(const std::string& message) {
    cJSON* json = cJSON_Parse(message.c_str());
    if (json == nullptr) {
        ESP_LOGE(TAG, "Failed to parse MCP message: %s", message.c_str());
        return;
    }
    ParseMessage(json);
    cJSON_Delete(json);
}

void McpServer::ParseCapabilities(const cJSON* capabilities) {
    auto vision = cJSON_GetObjectItem(capabilities, "vision");
    if (cJSON_IsObject(vision)) {
        auto url = cJSON_GetObjectItem(vision, "url");
        auto token = cJSON_GetObjectItem(vision, "token");
        if (cJSON_IsString(url)) {
            auto camera = Board::GetInstance().GetCamera();
            if (camera) {
                std::string url_str = std::string(url->valuestring);
                std::string token_str;
                if (cJSON_IsString(token)) {
                    token_str = std::string(token->valuestring);
                }
                camera->SetExplainUrl(url_str, token_str);
            }
        }
    }
}

void McpServer::ParseMessage(const cJSON* json) {
    // Check JSONRPC version
    auto version = cJSON_GetObjectItem(json, "jsonrpc");
    if (version == nullptr || !cJSON_IsString(version) || strcmp(version->valuestring, "2.0") != 0) {
        ESP_LOGE(TAG, "Invalid JSONRPC version: %s", version ? version->valuestring : "null");
        return;
    }
    
    // Check method
    auto method = cJSON_GetObjectItem(json, "method");
    if (method == nullptr || !cJSON_IsString(method)) {
        ESP_LOGE(TAG, "Missing method");
        return;
    }
    
    auto method_str = std::string(method->valuestring);
    if (method_str.find("notifications") == 0) {
        return;
    }
    
    // Check params
    auto params = cJSON_GetObjectItem(json, "params");
    if (params != nullptr && !cJSON_IsObject(params)) {
        ESP_LOGE(TAG, "Invalid params for method: %s", method_str.c_str());
        return;
    }

    auto id = cJSON_GetObjectItem(json, "id");
    if (id == nullptr || !cJSON_IsNumber(id)) {
        ESP_LOGE(TAG, "Invalid id for method: %s", method_str.c_str());
        return;
    }
    auto id_int = id->valueint;
    
    if (method_str == "initialize") {
        if (cJSON_IsObject(params)) {
            auto capabilities = cJSON_GetObjectItem(params, "capabilities");
            if (cJSON_IsObject(capabilities)) {
                ParseCapabilities(capabilities);
            }
        }
        auto app_desc = esp_app_get_description();
        std::string message = "{\"protocolVersion\":\"2024-11-05\",\"capabilities\":{\"tools\":{}},\"serverInfo\":{\"name\":\"" BOARD_NAME "\",\"version\":\"";
        message += app_desc->version;
        message += "\"}}";
        ReplyResult(id_int, message);
    } else if (method_str == "tools/list") {
        std::string cursor_str = "";
        if (params != nullptr) {
            auto cursor = cJSON_GetObjectItem(params, "cursor");
            if (cJSON_IsString(cursor)) {
                cursor_str = std::string(cursor->valuestring);
            }
        }
        GetToolsList(id_int, cursor_str);
    } else if (method_str == "tools/call") {
        if (!cJSON_IsObject(params)) {
            ESP_LOGE(TAG, "tools/call: Missing params");
            ReplyError(id_int, "Missing params");
            return;
        }
        auto tool_name = cJSON_GetObjectItem(params, "name");
        if (!cJSON_IsString(tool_name)) {
            ESP_LOGE(TAG, "tools/call: Missing name");
            ReplyError(id_int, "Missing name");
            return;
        }
        auto tool_arguments = cJSON_GetObjectItem(params, "arguments");
        if (tool_arguments != nullptr && !cJSON_IsObject(tool_arguments)) {
            ESP_LOGE(TAG, "tools/call: Invalid arguments");
            ReplyError(id_int, "Invalid arguments");
            return;
        }
        auto stack_size = cJSON_GetObjectItem(params, "stackSize");
        if (stack_size != nullptr && !cJSON_IsNumber(stack_size)) {
            ESP_LOGE(TAG, "tools/call: Invalid stackSize");
            ReplyError(id_int, "Invalid stackSize");
            return;
        }
        DoToolCall(id_int, std::string(tool_name->valuestring), tool_arguments, stack_size ? stack_size->valueint : DEFAULT_TOOLCALL_STACK_SIZE);
    } else {
        ESP_LOGE(TAG, "Method not implemented: %s", method_str.c_str());
        ReplyError(id_int, "Method not implemented: " + method_str);
    }
}

void McpServer::ReplyResult(int id, const std::string& result) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id) + ",\"result\":";
    payload += result;
    payload += "}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::ReplyError(int id, const std::string& message) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id);
    payload += ",\"error\":{\"message\":\"";
    payload += message;
    payload += "\"}}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::GetToolsList(int id, const std::string& cursor) {
    const int max_payload_size = 8000;
    std::string json = "{\"tools\":[";
    
    bool found_cursor = cursor.empty();
    auto it = tools_.begin();
    std::string next_cursor = "";
    
    while (it != tools_.end()) {
        // 如果我们还没有找到起始位置，继续搜索
        if (!found_cursor) {
            if ((*it)->name() == cursor) {
                found_cursor = true;
            } else {
                ++it;
                continue;
            }
        }
        
        // 添加tool前检查大小
        std::string tool_json = (*it)->to_json() + ",";
        if (json.length() + tool_json.length() + 30 > max_payload_size) {
            // 如果添加这个tool会超出大小限制，设置next_cursor并退出循环
            next_cursor = (*it)->name();
            break;
        }
        
        json += tool_json;
        ++it;
    }
    
    if (json.back() == ',') {
        json.pop_back();
    }
    
    if (json.back() == '[' && !tools_.empty()) {
        // 如果没有添加任何tool，返回错误
        ESP_LOGE(TAG, "tools/list: Failed to add tool %s because of payload size limit", next_cursor.c_str());
        ReplyError(id, "Failed to add tool " + next_cursor + " because of payload size limit");
        return;
    }

    if (next_cursor.empty()) {
        json += "]}";
    } else {
        json += "],\"nextCursor\":\"" + next_cursor + "\"}";
    }
    
    ReplyResult(id, json);
}

void McpServer::DoToolCall(int id, const std::string& tool_name, const cJSON* tool_arguments, int stack_size) {
    auto tool_iter = std::find_if(tools_.begin(), tools_.end(), 
                                 [&tool_name](const McpTool* tool) { 
                                     return tool->name() == tool_name; 
                                 });
    
    if (tool_iter == tools_.end()) {
        ESP_LOGE(TAG, "tools/call: Unknown tool: %s", tool_name.c_str());
        ReplyError(id, "Unknown tool: " + tool_name);
        return;
    }

    PropertyList arguments = (*tool_iter)->properties();
    try {
        for (auto& argument : arguments) {
            bool found = false;
            if (cJSON_IsObject(tool_arguments)) {
                auto value = cJSON_GetObjectItem(tool_arguments, argument.name().c_str());
                if (argument.type() == kPropertyTypeBoolean && cJSON_IsBool(value)) {
                    argument.set_value<bool>(value->valueint == 1);
                    found = true;
                } else if (argument.type() == kPropertyTypeInteger && cJSON_IsNumber(value)) {
                    argument.set_value<int>(value->valueint);
                    found = true;
                } else if (argument.type() == kPropertyTypeString && cJSON_IsString(value)) {
                    argument.set_value<std::string>(value->valuestring);
                    found = true;
                }
            }

            if (!argument.has_default_value() && !found) {
                ESP_LOGE(TAG, "tools/call: Missing valid argument: %s", argument.name().c_str());
                ReplyError(id, "Missing valid argument: " + argument.name());
                return;
            }
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "tools/call: %s", e.what());
        ReplyError(id, e.what());
        return;
    }

    // Start a task to receive data with stack size
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = "tool_call";
    cfg.stack_size = stack_size;
    cfg.prio = 1;
    esp_pthread_set_cfg(&cfg);

    // Use a thread to call the tool to avoid blocking the main thread
    tool_call_thread_ = std::thread([this, id, tool_iter, arguments = std::move(arguments)]() {
        try {
            ReplyResult(id, (*tool_iter)->Call(arguments));
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "tools/call: %s", e.what());
            ReplyError(id, e.what());
        }
    });
    tool_call_thread_.detach();
}


void McpServer::SetCurrentUserTWeight(float tweight) {
    current_user_tweight_ = tweight;
    ESP_LOGI(TAG,
             "SetCurrentUserTWeight: this=%p hw=%.3f",
             this,
             static_cast<double>(tweight));
}


