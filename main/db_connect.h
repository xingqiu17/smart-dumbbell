#ifndef DB_CONNECT_H
#define DB_CONNECT_H

#include "esp_err.h"
#include "esp_http_client.h"
#include <string>
#include <vector>
#include <utility>

/**
 * 与后端接口字段对应的 C++ 结构体
 */
struct User {
    int         userId;
    std::string account;
    std::string name;
    int         gender;
    std::string birthday;    // ISO 格式 yyyy-MM-dd
    float       height;
    float       weight;
    int         aim;
    float       hwWeight;
};



/**
 * @brief 将 JSON 字符串解析为 User 结构
 *
 * @param json   输入：JSON 字符串
 * @param user   输出：解析得到的 User 对象
 * @return true  解析成功
 * @return false 解析失败
 */
bool parse_user_json(const std::string& json, User& user);

/**
 * @brief 根据用户 ID 获取 User 结构
 *
 * 该函数内部会拼接 URL（例如 "http://<host>/api/v1/users/{id}"），
 * 调用 http_get 拉取数据，再用 parse_user_json 解析成 User。
 *
 * @param userId    要查询的用户 ID
 * @param out_user  输出：解析得到的 User 对象
 * @return true     获取并解析成功
 * @return false    获取或解析失败
 */

/**
 * 通用 HTTP 请求（支持 GET/POST/PUT/PATCH/DELETE，自动收集 chunked 响应）
 * @param method        HTTP_METHOD_GET / POST / PUT / PATCH / DELETE
 * @param url           完整 URL
 * @param body          可为 nullptr；非空时作为请求体发送
 * @param out_body      输出：响应体（可能为空，例如 204）
 * @param content_type  可为 nullptr；非空时设置 Content-Type
 * @param extra_headers 可为 nullptr；额外请求头（键值对）
 * @return ESP_OK 表示 2xx；否则返回错误码
 */
esp_err_t http_request(esp_http_client_method_t method,
                       const std::string& url,
                       const std::string* body,
                       std::string& out_body,
                       const char* content_type = nullptr,
                       const std::vector<std::pair<std::string,std::string>>* extra_headers = nullptr);





bool getUserInfo(int userId, User& out_user);


#endif // DB_CONNECT_H
