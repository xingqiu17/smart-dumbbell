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

    // 更新昵称
    bool UpdateName(int uid, const std::string& name);

    // 更新身体数据：birthday(YYYY-MM-DD)、height(cm)、weight(kg)、gender(0/1/2)
    bool UpdateBody(int uid, const std::string& birthday, float height, float weight, int gender);

    // 更新训练相关：aim 与 hwWeight(kg，可带小数)
    bool UpdateTrainData(int uid, int aim, float hwWeight);


    /** 用指定 uid 返回 JSON */
    std::string GetUserDataJson(int userId);

    /** 用当前已绑定 uid 返回 JSON；若未绑定返回 {"success":false,…} */
    std::string GetUserDataJson();


    // ===== 训练计划 =====

    // —— 查询（首选）
    bool GetDayPlans(int userId, const std::string& date, std::string& out_json);

    // —— 可选：便捷封装，使用当前 uid（内联）
    bool GetDayPlans(const std::string& date, std::string& out_json) {
        if (current_user_id_ < 0) return false;
        return GetDayPlans(current_user_id_, date, out_json);
    }

    // —— 创建（含内部先查后建的幂等保护）
    bool CreateDayPlan(int userId, const std::string& date,
                       const std::string& items_json, std::string& out_json);

    // —— 可选：便捷封装，使用当前 uid（内联）
    bool CreateDayPlan(const std::string& date,
                       const std::string& items_json, std::string& out_json) {
        if (current_user_id_ < 0) { out_json = R"({"success":false,"message":"user not bound"})"; return false; }
        return CreateDayPlan(current_user_id_, date, items_json, out_json);
    }


     // ===== 训练记录 =====

    // —— 创建：一次性保存 session + items + works
    bool CreateDayRecord(int userId, const std::string& date,
                         const std::string& items_json, std::string& out_json);

    // —— 便捷封装：使用当前 uid
    bool CreateDayRecord(const std::string& date,
                         const std::string& items_json, std::string& out_json) {
        if (current_user_id_ < 0) { out_json = R"({"success":false,"message":"user not bound"})"; return false; }
        return CreateDayRecord(current_user_id_, date, items_json, out_json);
    }


    // 直接按 sessionId 将计划标记为完成
    bool MarkPlanCompleteById(int sessionId);

private:
    RemoteDataService() = default;
    RemoteDataService(const RemoteDataService&)            = delete;
    RemoteDataService& operator=(const RemoteDataService&) = delete;

    int current_user_id_ = -1;   // -1 表示尚未绑定
};

#endif // REMOTE_DATA_SERVICE_H
