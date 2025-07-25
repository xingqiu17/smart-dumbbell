#ifndef DB_CONNECT_H
#define DB_CONNECT_H

#include "esp_err.h"
#include <string>

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
 * @brief 执行 HTTP GET 并返回响应体字符串
 *
 * @param url        请求 URL
 * @param out_body   输出：服务器返回的响应体
 * @return ESP_OK    表示成功，否则返回相应的错误码
 */
esp_err_t http_get(const std::string& url, std::string& out_body);

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
bool getUserInfo(int userId, User& out_user);


#endif // DB_CONNECT_H
