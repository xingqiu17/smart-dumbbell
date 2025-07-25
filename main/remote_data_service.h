#ifndef REMOTE_DATA_SERVICE_H
#define REMOTE_DATA_SERVICE_H

#include <string>
#include "db_connect.h"   // User 结构体 & getUserInfo()

/**
 * 统一管理与后端 REST API 的所有交互。
 * 目前只封装了用户信息接口；后续可继续添加训练计划、记录等方法。
 */
class RemoteDataService {
public:
    /** 获取单例实例 */
    static RemoteDataService& GetInstance();

    /* -------------------------------------------------------------- */
    /* 会话级用户 ID                                                   */
    /* -------------------------------------------------------------- */
    void SetCurrentUserId(int id)          { current_user_id_ = id; }
    int  GetCurrentUserId() const          { return current_user_id_; }

    /* -------------------------------------------------------------- */
    /* 用户相关                                                        */
    /* -------------------------------------------------------------- */

    /** 拉取用户信息并填充结构体（显式传 uid） */
    bool GetUserInfo(int userId, User& out_user);

    /** 用指定 uid 返回 JSON */
    std::string GetUserDataJson(int userId);

    /** 用当前已绑定 uid 返回 JSON；若未绑定返回 {"success":false,…} */
    std::string GetUserDataJson();

private:
    RemoteDataService() = default;
    RemoteDataService(const RemoteDataService&)            = delete;
    RemoteDataService& operator=(const RemoteDataService&) = delete;

    int current_user_id_ = -1;   // -1 表示尚未绑定
};

#endif // REMOTE_DATA_SERVICE_H
