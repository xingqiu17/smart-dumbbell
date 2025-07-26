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

#define TAG "MCP"

#define DEFAULT_TOOLCALL_STACK_SIZE 6144






McpServer::McpServer() {
}

McpServer::~McpServer() {
    for (auto tool : tools_) {
        delete tool;
    }
    tools_.clear();
}

void McpServer::AddCommonTools() {
    // To speed up the response time, we add the common tools to the beginning of
    // the tools list to utilize the prompt cache.
    // Backup the original tools list and restore it after adding the common tools.
    auto original_tools = std::move(tools_);
    auto& board = Board::GetInstance();



    // 获取用户数据工具
    AddTool("self.user.get_data",
        "When the user asks for user-related data, use this tool to reply including gender, height, weight, birthday, and current training weight data.",
        PropertyList({ Property("userId", kPropertyTypeInteger) }),
        [this](const PropertyList&) -> ReturnValue {
            int uid = this->current_user_id_;
            ESP_LOGI(TAG, "Tool self.user.get_data using stored userId=%d", uid);
            return RemoteDataService::GetInstance().GetUserDataJson(uid);
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
            auto name = props["name"].value<std::string>();
            ESP_LOGI(TAG, "set_name uid=%d, name=%s", uid, name.c_str());
            bool ok = RemoteDataService::GetInstance().UpdateName(uid, name);
            return ok ? R"({"success":true})" : R"({"success":false})";
        });

    // 修改身体数据：birthday(YYYY-MM-DD)、height(cm)、weight(kg)、gender(0/1/2)
    // 说明：当前 Property 仅支持 int/string/bool，为支持小数，height/weight 用 string 传，如 "172.5"
    AddTool("self.user.set_body",
        "Update user's body data.Include birthday,height,weight and gender.About gender, 0 is unkonwn,1 is male,2 is female.",
        PropertyList({
            Property("birthday", kPropertyTypeString),       // "YYYY-MM-DD"
            Property("height",   kPropertyTypeString),       // e.g. "172.0"
            Property("weight",   kPropertyTypeString),       // e.g. "60.5"
            Property("gender",   kPropertyTypeInteger, 0, 2) // 0..2
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";

            auto birthday = props["birthday"].value<std::string>();
            float height  = 0.f, weight = 0.f;
            try { height = std::stof(props["height"].value<std::string>()); } catch (...) {}
            try { weight = std::stof(props["weight"].value<std::string>()); } catch (...) {}
            int gender    = props["gender"].value<int>();

            ESP_LOGI(TAG, "set_body uid=%d, birthday=%s, h=%.2f, w=%.2f, g=%d",
                    uid, birthday.c_str(), height, weight, gender);

            bool ok = RemoteDataService::GetInstance().UpdateBody(uid, birthday, height, weight, gender);
            return ok ? R"({"success":true})" : R"({"success":false})";
        });


    // 修改训练目标/重量：aim、hwWeight(kg)
    // 说明：hwWeight 允许小数，因此用 string 传，例如 "11.5"
    AddTool("self.user.set_train_data",
        "Update user's training aim and hwWeight(kg).About aim: 0 is no aim,1 is arm,2 is shoulder, 3 is chest,4 is back,5 is leg",
        PropertyList({
            Property("aim",      kPropertyTypeInteger),
            Property("hwWeight", kPropertyTypeString)   // e.g. "11.5"
        }),
        [this](const PropertyList& props) -> ReturnValue {
            int uid = this->current_user_id_;
            if (uid <= 0) return R"({"success":false,"message":"no_user"})";

            int   aim = props["aim"].value<int>();
            float hw  = 0.f;
            try { hw = std::stof(props["hwWeight"].value<std::string>()); } catch (...) {}

            ESP_LOGI(TAG, "set_train_data uid=%d, aim=%d, hwWeight=%.2f", uid, aim, hw);

            bool ok = RemoteDataService::GetInstance().UpdateTrainData(uid, aim, hw);
            return ok ? R"({"success":true})" : R"({"success":false})";
        });

    

    // 查询某日的所有计划（只读，不创建）
    AddTool("self.plan.get_day",
        "Get all plans for the given date of the current user. "
        "Args: date (YYYY-MM-DD or 'today'). "
        "Return: JSON array. Use this ONLY for reading, never for creating.",
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
        "Each item MUST contain: type(int), number(int), tOrder(int). "
        "tWeight(float) is OPTIONAL — include it ONLY if the user explicitly specifies a weight; "
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

                const cJSON* jw = cJSON_GetObjectItemCaseSensitive(it, "tWeight");
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


