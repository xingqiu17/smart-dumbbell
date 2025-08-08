#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#include "audio_debugger.h"
#include "bmi2.h"
#include "bmi270.h"
#include "bmm150.h"
#include "i2c_bus.h"
#include <esp_rom_sys.h>
#include <esp_sleep.h>
#include "boards/atoms3r-echo-base/config.h"
#include <limits>
#include <vector>
#include <ctime>
#include "mdns.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "settings.h"
#include "remote_data_service.h" 
#include "websocket_srv.h" 
#include "http_client.h"
#include <mutex>



extern "C" {
#include "FusionAhrs.h"
#include "FusionMath.h"
}

using TrainingItem = Application::TrainingItem;

enum Axis {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
};

/* ====================  动作配置表  ==================== */
struct RepAlgoCfg {
    /* 计数—阈值及轴向 */
    float gyr_th;          // 开始/结束阈值 (rad/s)
    float gyr_hyst;        // 静止滞环     (rad/s)
    int   axis;            // 计数用哪根陀螺轴：0=X 1=Y 2=Z

    /* 随机森林模型指针 */
    const char* model; // 你在 rep_scorer.h 里声明的类
};

static constexpr float DEG2RAD = 0.0174532925f; // conversion factor from degrees to radians

/* ==== 配表：下标就是 Exercise 枚举 ==== */
static const RepAlgoCfg kAlgoCfg[] = {
/* EX_UNKNOWN */ { 0,0,1, nullptr },
/* EX_AIDBC   */ { 12.5f*DEG2RAD, 2.5f*DEG2RAD, 1, "repModel_DWC" /*占位*/ },
                 /* …… */
 /* EX_DWC     */ { 12.5f*DEG2RAD, 2.5f*DEG2RAD, 1, "repModel_DWC" },
 /* EX_DLR     */ { 10.0f*DEG2RAD, 2.0f*DEG2RAD, 0, "repModel_DLR" },
 /* EX_DSP     */ { 15.0f*DEG2RAD, 3.0f*DEG2RAD, 2, "repModel_DSP" },
 /* EX_45DBP   */ { 14.0f*DEG2RAD, 3.0f*DEG2RAD, 2, "repModel_45DBP" },
 /* EX_IDBC    */ { 12.5f*DEG2RAD, 2.5f*DEG2RAD, 1, "repModel_IDBC" },
};

// ========== 1.1 发送 / 接收通道 ==========
extern esp_err_t sendToServer(const char* json);         // websocket_srv.h 提供
extern bool      fetchScore(float& score_out);           // websocket_srv.cc 里实现

// ========== 1.2 rep 采样缓存 ==========
static constexpr int MAX_RAW_FRAMES = 400;               // ≤ 4 s@100 Hz
static float seqBuf[MAX_RAW_FRAMES][6];                  // gx gy gz ax ay az
static int   seqLen = 0;                                 // 当前写指针

static inline FusionQuaternion QuaternionConjugate(const FusionQuaternion& q)
{
    FusionQuaternion qc = { q.element.w,        // w 不变
                           -q.element.x,
                           -q.element.y,
                           -q.element.z };
    return qc;
}



//new
float rms_omega_y = 0.0f; // 用于计算平滑度的 Y 轴角速度 RMSb

RepReport rep_report = { EX_UNKNOWN, 0, 0.0f }; // 当前动作计数报告



static constexpr int MAX_ITEMS = 32;

static TrainingItem g_plan[MAX_ITEMS];
static int  g_plan_sz = 0;
static int  g_cur_idx = -1;

struct RecordItem {
    int type;        // 练习类型
    int num;         // 目标个数
    int tOrder;      // 组序号（1 起）
    float tWeight;   // 配重
    std::vector<float> scores; // 每次动作的得分
};
static std::vector<RecordItem> g_record_items;

static volatile bool g_item_running = false;   // 正在做某一组
static volatile bool g_need_next    = false;   // 某组完成后置位，驱动切换

static volatile bool g_in_rest      = false;   // 休息模式
static esp_timer_handle_t rest_timer_handle = nullptr;

//new

#define WAKE_WORD_APPLY 0 //唤醒词是否启用    0 不启用    1 启用

static bool ws_started = false;

/* —— AHRS & 运动识别 —— */
static FusionAhrs ahrs;               // 全局滤波器
/* ---------- 动作计数用四状态机 ---------- */
enum Phase { PHASE_IDLE, PHASE_UP, PHASE_TOP, PHASE_DOWN };
static Phase phase = PHASE_IDLE;
static float pMax, pMin, rMax, rMin;
static int rep_cnt=0;
bool is_rep_counting = false; // 是否正在计数

// 计划会话ID缓存（-1 表示未知）
static int g_plan_session_id = -1;

static std::mutex g_record_mtx;




// application.cc 里（建议在最前面 IMU 相关静态变量附近）
struct ImuFrame {
    float ax, ay, az;          // g
    float gx, gy, gz;          // rad/s
    uint64_t ts;               // µs 时间戳，可做节拍
};

static constexpr size_t IMU_BUF_LEN = 128;     // 100 Hz 采样 ≈1.28 s
static ImuFrame imuBuf[IMU_BUF_LEN];
static size_t   wrIdx   = 0;                   // 写指针



static int GetRestSeconds(int idx) {
    if (idx < 0 || idx >= g_plan_sz) return 0;
    return g_plan[idx].rest;
}

static std::string CurrentDateISO() {
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    char buf[11];
    strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
    return std::string(buf);
}

static void FinishTraining() {

    if (g_cur_idx >= 0 && g_cur_idx < g_plan_sz && g_cur_idx < (int)g_record_items.size()) {
    g_record_items[g_cur_idx].num = rep_cnt;
    }
    ESP_LOGI("TRAIN", "Plan finished: %d items done", g_plan_sz);
    cJSON* items = cJSON_CreateArray();
    for (const auto& it : g_record_items) {
        cJSON* obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(obj, "type",    it.type);
        cJSON_AddNumberToObject(obj, "num",     it.num);
        cJSON_AddNumberToObject(obj, "tOrder",  it.tOrder);
        cJSON_AddNumberToObject(obj, "tWeight", it.tWeight);
        int avg = 0;
        if (!it.scores.empty()) {
            float sum = 0.0f;
            for (float s : it.scores) sum += s;
            avg = static_cast<int>(sum / it.scores.size() + 0.5f);
        }
        cJSON_AddNumberToObject(obj, "avgScore", avg);

        cJSON* works = cJSON_CreateArray();
        for (size_t j = 0; j < it.scores.size(); ++j) {
            cJSON* w = cJSON_CreateObject();
            cJSON_AddNumberToObject(w, "acOrder", (int)j + 1);
            cJSON_AddNumberToObject(w, "score",   static_cast<int>(it.scores[j] + 0.5f));
            cJSON_AddItemToArray(works, w);
        }
        cJSON_AddItemToObject(obj, "works", works);
        cJSON_AddItemToArray(items, obj);
    }

    char* raw = cJSON_PrintUnformatted(items);
    std::string items_json = raw ? raw : "[]";
    if (raw) cJSON_free(raw);
    cJSON_Delete(items);

    std::string resp;
    auto& rds = RemoteDataService::GetInstance();
    std::string date = CurrentDateISO();
    if (rds.CreateDayRecord(date, items_json, resp)) {
        ESP_LOGI("TRAIN", "Upload training record ok");
    } else {
        ESP_LOGE("TRAIN", "Upload training record failed");
    }
    g_record_items.clear();

    if (g_plan_session_id > 0) {
            if (rds.MarkPlanCompleteById(g_plan_session_id)) {
                ESP_LOGI("TRAIN", "Plan session %d marked complete", g_plan_session_id);
            } else {
                ESP_LOGW("TRAIN", "Mark complete failed for session %d", g_plan_session_id);
            }
        } else {
            ESP_LOGW("TRAIN", "Skip mark-complete: no valid sessionId cached");
        }
        g_plan_session_id = -1; // 清除缓存

    auto display = Board::GetInstance().GetDisplay();
    display->ShowPage("main");
}



static void StartNextItem() {
    rep_cnt = 0;
    if (++g_cur_idx >= g_plan_sz) {
        FinishTraining();
        return;
    }

    const TrainingItem& it = g_plan[g_cur_idx];
    // 枚举值与你的 JSON type 对齐，直接强转即可
    rep_report.ex   = static_cast<Exercise>(it.type);
    rep_report.rep_idx = it.reps;   // 目标个数
    // 如需把配重下发给硬件/电机，可在此处使用 it.weight

    // 重置计数/状态机
    is_rep_counting = true;
    phase = PHASE_IDLE;
    // 清零你在 imu_task 内部用到的统计变量（若有文件内静态，可在此处重置）

    g_item_running = true;

    /* ---- 在这里刷新训练页 ---- */
    auto  display = Board::GetInstance().GetDisplay();
    const char* zh = ActionName(rep_report.ex);          // 你的中文映射
    display->UpdateExercise(zh, 0 /*已完成次数*/, 0.0f);
    display->ShowPage("workout");

    ESP_LOGI("TRAIN", "Start item #%d/%d: type=%d reps=%d weight=%.2f",
             g_cur_idx+1, g_plan_sz, it.type, it.reps, it.weight);
}

static void StartPlan() {
    std::lock_guard<std::mutex> lk(g_record_mtx);
    g_record_items.clear();
    for (int i = 0; i < g_plan_sz; ++i) {
        g_record_items.push_back({g_plan[i].type, 0, i + 1, g_plan[i].weight, {}});
    }
    g_cur_idx = -1;
    g_item_running = false;
    g_need_next = false;
    StartNextItem();
}


bool Application::StartTrainingFromItems(int sessionId, const std::vector<TrainingItem>& items) {
    // 1) 若正在休息，先停表
    if (g_in_rest && rest_timer_handle) {
        esp_timer_stop(rest_timer_handle);
        g_in_rest = false;
    }

    // 2) 基本校验
    if (items.empty() || items.size() > (size_t)MAX_ITEMS) {
        ESP_LOGE("TRAIN", "StartTrainingFromItems: bad items size=%d", (int)items.size());
        return false;
    }

    // 3) 写入计划、缓存 sessionId
    {
        std::lock_guard<std::mutex> lk(g_record_mtx);
        g_plan_sz = (int)items.size();
        for (int i = 0; i < g_plan_sz; ++i) {
            g_plan[i] = items[i]; // TrainingItem {type, reps, weight}
        }
        g_plan_session_id = sessionId;
    }

    // 4) 启动训练
    StartPlan();  // static 本地可见，OK
    ESP_LOGI("TRAIN", "StartTrainingFromItems: sid=%d sets=%d", sessionId, g_plan_sz);
    return true;
}


static void RestTimerCallback(void* arg) {
    g_in_rest = false;
    StartNextItem();
}

static void EnterRest() {
    std::lock_guard<std::mutex> lk(g_record_mtx);
    if (g_cur_idx >= 0 && g_cur_idx < (int)g_record_items.size()) {
        g_record_items[g_cur_idx].num = rep_cnt; // 同步次数
    }

    /* 最后一组做完就直接结束，不休息 */
    if (g_cur_idx >= g_plan_sz - 1) {
        FinishTraining();
        return;
    }

    /* === 根据 plan 中的 rest 秒数启动定时 === */
    int rest_sec = GetRestSeconds(g_cur_idx);
    if (rest_sec <= 0) {          // 0 表示跳过休息
        StartNextItem();
        return;
    }

    g_in_rest = true;

    /* ---- 切到 Pause 页 ---- */
    auto display = Board::GetInstance().GetDisplay();
    display->UpdatePause(
        rep_report.ex,            // 当前动作 id
        rep_report.rep_idx,       // 目标次数
        rest_sec                  // 倒计时秒
    );

    /* ---- 启动一枪定时器 ---- */
    uint64_t rest_us = (uint64_t)rest_sec * 1'000'000ULL;
    if (!rest_timer_handle) {
        esp_timer_create_args_t args = {
            .callback = &RestTimerCallback,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "rest_timer",
            .skip_unhandled_events = true
        };
        esp_timer_create(&args, &rest_timer_handle);
    }
    esp_timer_stop(rest_timer_handle);
    esp_timer_start_once(rest_timer_handle, rest_us);

    ESP_LOGI("TRAIN", "Resting for %d seconds", rest_sec);
}


#if CONFIG_USE_AUDIO_PROCESSOR
#include "afe_audio_processor.h"
#else
#include "no_audio_processor.h"
#endif

#if CONFIG_USE_AFE_WAKE_WORD
#include "afe_wake_word.h"
#elif CONFIG_USE_ESP_WAKE_WORD
#include "esp_wake_word.h"
#else
#include "no_wake_word.h"
#endif

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"
void websocket_server_start();

#define BMM150_I2C_ADDR_SDO0  0x10   // SDO=0 → 0x10
#define BMM150_I2C_ADDR_SDO1  0x11   // SDO=1 → 0x11
#define AUX_READ_LEN_MAX      64
QueueHandle_t Application::s_imuQueue = nullptr;   // 默认空
QueueHandle_t Application::s_magQueue = nullptr;

QueueHandle_t Application::s_jsonQueue = nullptr;
QueueHandle_t Application::GetJsonQueue() { return s_jsonQueue; }
Application& Application::GetInstance() {
    static Application instance;
    return instance;
}

void send_rep_json(const RepReport& rp)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "event", "rep_data");
    cJSON_AddNumberToObject(root, "exercise", rp.ex);
    cJSON_AddNumberToObject(root, "rep",      rp.rep_idx);
    cJSON_AddNumberToObject(root, "score",    rp.score);

    char* msg = cJSON_PrintUnformatted(root);      // 动态分配
    esp_err_t err = sendToClient(msg);                // 你 websocket 的封装
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "发送 rep_data JSON 失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "发送 rep_data JSON: %s", msg);
    }
    cJSON_free(msg);
    cJSON_Delete(root);
}




/*********************  头文件 & 宏保持不变  **************************/

static i2c_bus_handle_t        i2c_bus     = nullptr;
static i2c_bus_device_handle_t i2c_dev_bmi = nullptr;   // 仅 BMI270

/* ---------- BMI270 I²C 回调 ---------- */
static int8_t bmi2_i2c_read (uint8_t reg, uint8_t *data, uint32_t len, void*)
{ return i2c_bus_read_bytes(i2c_dev_bmi, reg, len, data)==ESP_OK ? 0:-1; }

static int8_t bmi2_i2c_write(uint8_t reg, const uint8_t *dat, uint32_t len, void*)
{ return i2c_bus_write_bytes(i2c_dev_bmi, reg, len, dat)==ESP_OK ? 0:-1; }

static void   bmi2_delay_us(uint32_t t, void*) { esp_rom_delay_us(t); }

/* --------- AUX ↔ BMM150 单字节适配器 ---------- */
static int8_t bmm150_aux_i2c_read(uint8_t reg, uint8_t *data,
                                  uint32_t len, void *ptr)
{
    for (uint32_t i = 0; i < len; i++) {
        if (bmi2_read_aux_man_mode(reg + i, &data[i], 1,
                                   (bmi2_dev *)ptr) != BMI2_OK)
            return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

static int8_t bmm150_aux_i2c_write(uint8_t reg, const uint8_t *data,
                                   uint32_t len, void *ptr)
{
    for (uint32_t i = 0; i < len; i++) {
        if (bmi2_write_aux_man_mode(reg + i, (uint8_t *)&data[i], 1,
                                    (bmi2_dev *)ptr) != BMI2_OK)
            return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}




// I2C 初始化
void Application::init_i2c()
{
    i2c_config_t cfg{};
    cfg.mode           = I2C_MODE_MASTER;
    cfg.sda_io_num     = 2;
    cfg.scl_io_num     = 1;
    cfg.sda_pullup_en  = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en  = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400000;

    i2c_bus = i2c_bus_create(I2C_NUM_0, &cfg);
    ESP_ERROR_CHECK(i2c_bus ? ESP_OK : ESP_FAIL);

    // BMI270 (addr 0x68)
    i2c_dev_bmi = i2c_bus_device_create(i2c_bus, BMI270_I2C_ADDRESS, 0);

    ESP_ERROR_CHECK(i2c_dev_bmi  ? ESP_OK : ESP_FAIL);
}         

/*********************  传感器句柄  **************************/
static struct bmi2_dev   bmi;
static struct bmm150_dev bmm;

static void dump_bmm150_regs()
{
    auto rb = [](uint8_t reg)   /* 读一个字节并返回 */
    {
        uint8_t v = 0;
        bmi2_read_aux_man_mode(reg, &v, 1, &bmi);
        return v;
    };

    uint8_t chip_id = rb(0x40);
    uint8_t pwr_ctl = rb(0x4B);
    uint8_t odr_rep = rb(0x4C);
    uint8_t rep_xy  = rb(0x51);
    uint8_t rep_z   = rb(0x52);

    uint8_t raw[8];
    for (int i = 0; i < 8; ++i) raw[i] = rb(0x42 + i);

    ESP_LOGI("BMM-DUMP",
             "ID=0x%02X  Pwr=0x%02X  4C=0x%02X  51=0x%02X  52=0x%02X",
             chip_id, pwr_ctl, odr_rep, rep_xy, rep_z);

    ESP_LOGI("BMM-DUMP",
             "Raw[42-49] = %02X %02X %02X %02X  %02X %02X %02X %02X",
             raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7]);
}

void Application::init_sensors()
{
   /* ---------- 1. BMI270 + I²C 基础 ---------- */
    bmi.intf           = BMI2_I2C_INTF;
    bmi.read           = bmi2_i2c_read;
    bmi.write          = bmi2_i2c_write;
    bmi.delay_us       = bmi2_delay_us;
    ESP_ERROR_CHECK(bmi270_init(&bmi));

    // ② 只在第一次调用时初始化 AHRS
FusionAhrsInitialise(&ahrs);          // 内部自带默认参数 :contentReference[oaicite:1]{index=1}

FusionAhrsSettings settings = {
    .convention            = FusionConventionNwu, // NWU/NED/ENU 三选一
    .gain                  = 0.8f,               // 相当于 Madgwick β :contentReference[oaicite:2]{index=2}
    .gyroscopeRange        = 250.0f,              // °/s，决定“角速度恢复”阈值
    .accelerationRejection = 10.0f,               // 动态拒绝直线加速度 (°)
    .magneticRejection     = 15.0f,               // 动态拒绝磁畸变   (°)
    .recoveryTriggerPeriod = 5,                   // 5 个采样点触发恢复
};
FusionAhrsSetSettings(&ahrs, &settings);          // 应用自定义参数 :contentReference[oaicite:3]{index=3}

    /* 2. 先把 ACC / GYR / AUX 都 enable，AUX 进入“手动模式” */
    bmi2_sens_config cfg[3]{};
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    cfg[2].type = BMI2_AUX;

    /* 2-1) 给 ACC / GYR 一个最基本的 ODR（100 Hz）——随意 */
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

    /* 2-2) AUX 手动模式参数（完全抄官方例程）*/
    auto &aux = cfg[2].cfg.aux;
    aux.aux_en          = BMI2_ENABLE;
    aux.manual_en       = BMI2_ENABLE;         // **先手动**
    aux.fcu_write_en    = BMI2_ENABLE;
    aux.i2c_device_addr = BMM150_I2C_ADDR_SDO0;/* 0x10 */
    aux.man_rd_burst    = BMI2_AUX_READ_LEN_3; // 8 byte 连读
    aux.read_addr       = BMM150_REG_DATA_X_LSB; /* 0x42 */
    aux.odr             = BMI2_AUX_ODR_100HZ;

    ESP_ERROR_CHECK(bmi270_set_sensor_config(cfg, 3, &bmi));
    ESP_ERROR_CHECK(bmi270_sensor_enable((uint8_t[]){BMI2_ACCEL,BMI2_GYRO,BMI2_AUX},3,&bmi));

    /**************************************************************************/
    /* ---------- 3. 手动模式下跑 bmm150_init() ---------- */
    bmm.intf     = BMM150_I2C_INTF;            // 仍写 I²C
    bmm.read     = bmm150_aux_i2c_read;
    bmm.write    = bmm150_aux_i2c_write;
    bmm.delay_us = bmi2_delay_us;
    bmm.intf_ptr = &bmi;

    ESP_ERROR_CHECK(bmm150_init(&bmm));        // 现在能读到 0x32

    /* 3-1) 用 ENHANCED 预设 / NORMAL power */
    bmm150_settings bmm_cfg{};
    bmm_cfg.preset_mode = BMM150_PRESETMODE_ENHANCED;
    ESP_ERROR_CHECK(bmm150_set_presetmode(&bmm_cfg, &bmm));
    bmm_cfg.pwr_mode    = BMM150_POWERMODE_NORMAL;
    ESP_ERROR_CHECK(bmm150_set_op_mode(&bmm_cfg, &bmm));

    /**************************************************************************/
    /* ---------- 4. 切回 AUTO 模式 ---------- */
    aux.manual_en    = BMI2_DISABLE;           // **关手动，进自动**
    aux.fcu_write_en = BMI2_DISABLE;
    aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;    // 每次 8 字节
    aux.read_addr    = BMM150_REG_DATA_X_LSB;  // 0x42
    ESP_ERROR_CHECK(bmi270_set_sensor_config(&cfg[2],1,&bmi));

    ESP_LOGI(TAG,"BMI270-AUX + BMM150 init OK (auto mode)");

    dump_bmm150_regs();
    if (s_imuQueue == nullptr)            // 只建一次
    {
        s_imuQueue = xQueueCreate(/*length*/ 1, sizeof(bmi2_sens_data));
        configASSERT(s_imuQueue);          // 创建失败直接 reset
    }
    if (s_magQueue == nullptr)
    {
        s_magQueue = xQueueCreate(/*length*/ 1, sizeof(float[3]));
        configASSERT(s_magQueue);
    }
}





bool Application::GetLatestImu(bmi2_sens_data& out)
{
    if (s_imuQueue == nullptr) return false;

    /* 0 tick -> 立即返回，不阻塞 */
    if (xQueueReceive(s_imuQueue, &out, 0) == pdTRUE)
        return true;

    return false;   // 队列暂时空
}


bool Application::GetLatestMag(float out[3])
{
    if (s_magQueue == nullptr) return false;
    /* 立即返回，不阻塞；拷贝 3 个 float */
    return xQueueReceive(s_magQueue, out, 0) == pdTRUE;
}




void Application::imu_task(void* arg)
{
    bmi2_sens_data imu{};      // 本地缓冲
    bmm150_mag_data mag{}; 
    float          mag_uT[3];

    while (true)
    {   
        if(is_rep_counting){
        if (bmi2_get_sensor_data(&imu, &bmi) == BMI2_OK) {
            /* ====== ① 推进队列 ====== */
            if (s_imuQueue) { xQueueOverwrite(s_imuQueue, &imu); }
                        /* ====== 姿态解算 ====== */
            constexpr float ACC_LSB = 16384.0f;             // ±2 g
            constexpr float GYR_LSB = 131.0f;               // ±250 °/s
            constexpr float DEG2RAD = 0.0174532925f;

            // 1) 原始计数 -> 物理量
            float ax =  imu.acc.x / ACC_LSB ;
            float ay =  imu.acc.y / ACC_LSB ;
            float az =  imu.acc.z / ACC_LSB ;
            float gx = (imu.gyr.x / GYR_LSB) * DEG2RAD;
            float gy = (imu.gyr.y / GYR_LSB) * DEG2RAD;
            float gz = (imu.gyr.z / GYR_LSB) * DEG2RAD;
            constexpr float LSB2UT = 0.0625f;
            if (bmm150_aux_mag_data(imu.aux_data, &mag, &bmm) == BMM150_OK) {
            mag_uT[0] = mag.x * LSB2UT;
            mag_uT[1] = mag.y * LSB2UT;
            mag_uT[2] = mag.z * LSB2UT;
            }

            uint64_t now_us = esp_timer_get_time();
            static uint64_t prev_us = now_us;
            float dt = (now_us - prev_us) * 1e-6f;
            prev_us = now_us;
                        
            


            // 2) 更新四元数（9 轴）
        FusionVector gyro = {.axis = {gx,  gy,  gz}};
        FusionVector acc  = {.axis = {ax,    ay,    az}};
        FusionVector mag1  = {.axis = {mag_uT[0],   mag_uT[1],   mag_uT[2]}};
        FusionAhrsUpdate(&ahrs, gyro, acc, mag1, dt);

                imuBuf[wrIdx] = { imu.acc.x/ACC_LSB,
                        imu.acc.y/ACC_LSB,
                        imu.acc.z/ACC_LSB,
                        gx, gy, gz,
                        now_us };
        wrIdx = (wrIdx + 1) % IMU_BUF_LEN;
 
        
                        // ***** 在读完 ax…gz 之后 *****
        if (seqLen < MAX_RAW_FRAMES) {
            seqBuf[seqLen][0] = gx;  
            seqBuf[seqLen][1] = gy;
            seqBuf[seqLen][2] = gz;
            seqBuf[seqLen][3] = ax;
            seqBuf[seqLen][4] = ay;
            seqBuf[seqLen][5] = az;
            ++seqLen;
        }

            // 3) 取俯仰角
                        /* ---------- 计数 & 分类核心 ---------- */
            // constexpr float GYR_TH   = 12.5f * DEG2RAD;   
            // constexpr float GYR_HYST =  2.5f * DEG2RAD;   
            static uint64_t cycle_start_us = 0;           // 回合开始时间
                        /* ---- 本 rep 的临时统计量（static 保存在多次循环之间） ---- */
            static float     peak_gyr_y   = 0.0f;    // |gy| 峰值
            static float     sum_gyr_y    = 0.0f;    // |gy| 积分
            static uint32_t  sample_cnt   = 0;       // 计数
            static float     peak_acc_mag = 0.0f;    // √(ax²+ay²+az²) 峰值
            static uint64_t  t_phase_start = 0;

            const RepAlgoCfg& cfg = kAlgoCfg[rep_report.ex];
            const float  GYR_TH   = cfg.gyr_th;// 上升 / 下降阈值
            const float  GYR_HYST = cfg.gyr_hyst;// 静止滞环

                        /* 选取本次采样做计数的原始角速度 --------- */
            float gyro_raw_all[3] = {
                (imu.gyr.x / GYR_LSB) * DEG2RAD,
                (imu.gyr.y / GYR_LSB) * DEG2RAD,
                (imu.gyr.z / GYR_LSB) * DEG2RAD
            };
            float gy_raw = gyro_raw_all[cfg.axis];

            /* 做一次一阶 LPF */
            static float gy_lp = 0.0f;
            gy_lp = 0.8f * gy_lp + 0.2f * gy_raw;

            /* 取实时 pitch / roll（°）*/
            FusionEuler eu = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            float pitch = eu.angle.pitch;
            float roll  = eu.angle.roll;
            
                        /* feed RMS 缓冲 —— 放在采样更新最底部 */
            static float win_omega2[50];              // 0.5 s 窗口（100 Hz）
            static int   win_pos = 0;

            win_omega2[win_pos] = gy_raw*gy_raw;      // 原始 Y 轴角速度平方
            win_pos = (win_pos + 1) % 50;

            float sum = 0;
            for (float v : win_omega2) sum += v;
            rms_omega_y = sqrtf(sum / 50) / DEG2RAD;  // 转回 °/s
            /* ---- 计算实时 RMS 平滑度 —— */

            /* ---- 本次采样的加速度模长 ---- */
            float acc_mag = sqrtf(ax*ax + ay*ay + az*az);   // ★★★ NEW
        
            /* ------------ 四状态机 ------------- */
            switch (phase) {
            case PHASE_IDLE:
                if (gy_lp > GYR_TH) {                    // 开始上举
                    phase = PHASE_UP;
                    pMax = pMin = pitch;                 // 清极值
                    rMax = rMin = roll ;
                    cycle_start_us = now_us;                    // 复位计时
                    t_phase_start = now_us;
                    peak_gyr_y = fabsf(gy_raw);

                                        /* ★★★ NEW: 清零统计量 */
                    peak_gyr_y   = fabsf(gy_raw);
                    sum_gyr_y    = fabsf(gy_raw);
                    sample_cnt   = 1;
                    peak_acc_mag = acc_mag;
                }
                break;

            case PHASE_UP:
                pMax = std::max(pMax, pitch);
                pMin = std::min(pMin, pitch);
                rMax = std::max(rMax, roll );
                rMin = std::min(rMin, roll );
                            /* ★★★ NEW: 累积统计 */
                peak_gyr_y   = std::max(peak_gyr_y, fabsf(gy_raw));
                sum_gyr_y   += fabsf(gy_raw);
                peak_acc_mag = std::max(peak_acc_mag, acc_mag);
                ++sample_cnt;
                if (fabsf(gy_lp) < GYR_HYST){             // 到顶
                    phase = PHASE_TOP;
                }
                break;

            case PHASE_TOP:
                if (gy_lp < -GYR_TH){                     // 开始下降
                    phase = PHASE_DOWN;
                    t_phase_start = now_us;
            }
                break;

            case PHASE_DOWN:
                pMax = std::max(pMax, pitch);
                pMin = std::min(pMin, pitch);
                rMax = std::max(rMax, roll );
                rMin = std::min(rMin, roll );
                                /* ★★★ NEW: 累积统计 */
                peak_gyr_y   = std::max(peak_gyr_y, fabsf(gy_raw));
                sum_gyr_y   += fabsf(gy_raw);
                peak_acc_mag = std::max(peak_acc_mag, acc_mag);
                ++sample_cnt;
                if (fabsf(gy_lp) < GYR_HYST) {           // 动作完成
                    float dP = pMax - pMin;
                    float dR = rMax - rMin;
                    rep_cnt++; // 计数
                    

                auto display = Board::GetInstance().GetDisplay();
                display->UpdateExercise(ActionName(rep_report.ex),   // 中文名
                                        rep_cnt,                     // 当前完成数
                                        0.9);                      // 即时分数





                    uint64_t rep_us   = now_us - cycle_start_us;   // 本次用时

                                /* ================= 生成 7 维特征 ================= */
                    float repDur_s = (now_us - cycle_start_us) * 1e-6f;               // rep 总时长
                    float tempo    = repDur_s;                                        // 你训练里若是 up/down 比例可改

                    float avgGyro  = (sample_cnt ? sum_gyr_y / sample_cnt : 0.0f);    // 平均 |gy|
                                    /* ------- 你原来的结束状态里，计算好每回合的统计量 -------- */
                    float romPitch  = dP;                  // 俯仰范围  (°)
                    float romRoll   = dR;                  // 滚转范围  (°)
                    float smooth    = rms_omega_y;         // 抖动指标(°/s)
                    float maxAcc    = peak_acc_mag;             // 例：回合峰值加速度(g)
                    float repDur    = repDur_s;                // 如果训练用了同一项可重复

                    float features[7] = {
                        romPitch,
                        romRoll,
                        tempo,
                        smooth,
                        avgGyro,
                        maxAcc,
                        repDur
                    };
                    float score = 0.0f; // 模型输出分数
                    int exercise = rep_report.ex; // 当前动作类型
                    /* ------- 调用模型 -------- */
                    
                    /* 你愿意的话再映射到 0–100 或做平滑 */
                    score = std::clamp(score, 0.f, 100.f);

                    if (g_cur_idx >= 0 && g_cur_idx < (int)g_record_items.size()) {
                        g_record_items[g_cur_idx].scores.push_back(score);
                        g_record_items[g_cur_idx].num = rep_cnt;
                    }

                    if (rep_cnt == rep_report.rep_idx) {
                        is_rep_counting = false;
                        g_item_running = false;
                        Application::GetInstance().Schedule([]() {
                            EnterRest();
                        });
                    }
                    
                    
                    /* ---------- 3.1 200×6 重采样 ---------- */
                    constexpr int TARGET_LEN = 200;
                    constexpr int MAX_JSON   = 12 * TARGET_LEN * 6 + 160;   // 15 KB 裁剪裕量
                    static char   jsonBuf[MAX_JSON];

                    char *p = jsonBuf;
                    int  n  = snprintf(                     // 先写头
                            p, MAX_JSON,
                            "{\"code\":\"%s\",\"rep\":%d,\"seq\":[",
                            kExerciseStr[rep_report.ex],    // ★ 确保字符串数组里和服务器端一致
                            rep_cnt);
                    if (n <= 0 || n >= MAX_JSON) return;    // 理论不会

                    p += n;                                 // p 指向下一写入位置

                    for (int k = 0; k < TARGET_LEN; ++k) {
                        float idx = k * (seqLen - 1.0f) / (TARGET_LEN - 1); // 连续坐标 0…len-1
                        int   i0  = (int)idx;
                        float t   = idx - i0;

                        float frame[6];
                        if (i0 >= seqLen - 1) {             // ★ 边界：最后一个点直接拷贝
                            memcpy(frame, seqBuf[seqLen - 1], sizeof(frame));
                        } else {
                            for (int c = 0; c < 6; ++c)
                                frame[c] = seqBuf[i0][c] * (1 - t) + seqBuf[i0 + 1][c] * t;
                        }

                        n = snprintf(p, MAX_JSON - (p - jsonBuf),
                                    "[%.5f,%.5f,%.5f,%.5f,%.5f,%.5f]%s",
                                    frame[0], frame[1], frame[2],
                                    frame[3], frame[4], frame[5],
                                    (k == TARGET_LEN - 1 ? "]}" : ","));
                        if (n <= 0 || n >= MAX_JSON - (p - jsonBuf)) {
                            ESP_LOGE("JSON", "jsonBuf overflow!");
                            return;                         // 保护：不发送不完整 JSON
                        }
                        p += n;
                    }

                    /* ---------- 3.2 发送 ---------- */
                    * p = '\0';                             // 保险：确认以 0 结尾
                    esp_err_t err = sendToServer(jsonBuf);
                    if (err != ESP_OK) {
                        ESP_LOGE("HTTP_SRV", "sendToServer failed: %s", esp_err_to_name(err));
                    }
                    else{
                        ESP_LOGI("HTTP_SRV", "sendToServer OK: ");
                    }

                    /* 清空缓存，准备下一 rep */
                    seqLen = 0;
                    
                    /* 生成报告 */
                    RepReport rp2client;
                    rp2client.ex       = rep_report.ex;
                    rp2client.rep_idx  = rep_cnt;
                    rp2client.score    = score;
                    send_rep_json(rp2client);


                    


                    /* 保存最近 N 次得分做滑动平均 */
                    static constexpr int N = 5;
                    static float score_hist[N] = {0};
                    static int   sh_idx = 0;

                    score_hist[sh_idx] = score;
                    sh_idx = (sh_idx + 1) % N;

                    float avg = 0;
                    for (int i=0;i<N;++i) avg += score_hist[i];
                    avg /= N;

                    ESP_LOGI("FIT",
                            "Rep=%d  act=%d  ΔP=%.1f  ΔR=%.1f  t=%.2f s  score=%.1f  avg=%.1f",
                            rep_cnt, rep_report.ex, dP, dR, rep_us/1e6f, score, avg);

                    phase = PHASE_IDLE;
                }
                break;
            }



            /* ④ —— 欧拉角 & 原始 9 轴同时打印 —— */
            static uint64_t lastPrint = 0;
            if (now_us - lastPrint > 200000) {          // 200 ms 一次
                lastPrint = now_us;
                FusionEuler eu = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                float yaw   = eu.angle.yaw;
                float pitch = eu.angle.pitch;
                float roll  = eu.angle.roll;

                ESP_LOGI("9DOF",
                        "Yaw/Pitch/Roll=%.1f/%.1f/%.1f  "
                        "ACC[g]=%.3f,%.3f,%.3f  "
                        "GYR[dps]=%.1f,%.1f,%.1f  "
                        "MAG[uT]=%.1f,%.1f,%.1f", 
                        yaw, pitch, roll,
                        imu.acc.x/ACC_LSB, imu.acc.y/ACC_LSB, imu.acc.z/ACC_LSB,
                        gx/DEG2RAD, gy/DEG2RAD, gz/DEG2RAD,
                        mag_uT[0], mag_uT[1], mag_uT[2]);
            }


            /* ====== ② 仍然打印日志 ====== */
            if (bmm150_aux_mag_data(imu.aux_data, &mag, &bmm) == BMM150_OK) {
                constexpr float LSB2UT = 0.0625f;
                mag_uT[0] = mag.x * LSB2UT;
                mag_uT[1] = mag.y * LSB2UT;
                mag_uT[2] = mag.z * LSB2UT;
                if (s_magQueue){
                    /* 直接写 3 * float, 注意长度要与队列创建时一致 */
                    xQueueOverwrite(s_magQueue, mag_uT);
                }
            }
            

        }
        }
        vTaskDelay(pdMS_TO_TICKS(10));      // 
    }
}







static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "audio_testing",
    "fatal_error",
    "invalid_state"
    "POWER_OFF"
};

Application::Application() {
    
    event_group_ = xEventGroupCreate();
    background_task_ = new BackgroundTask(4096 * 7);

    

#if CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

#if CONFIG_USE_AUDIO_PROCESSOR
    audio_processor_ = std::make_unique<AfeAudioProcessor>();
#else
    audio_processor_ = std::make_unique<NoAudioProcessor>();
#endif

#if CONFIG_USE_AFE_WAKE_WORD
    wake_word_ = std::make_unique<AfeWakeWord>();
#elif CONFIG_USE_ESP_WAKE_WORD
    wake_word_ = std::make_unique<EspWakeWord>();
#else
    wake_word_ = std::make_unique<NoWakeWord>();
#endif

    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void* arg) {
            Application* app = (Application*)arg;
            app->OnClockTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    if (background_task_ != nullptr) {
        delete background_task_;
    }
    vEventGroupDelete(event_group_);
}

// void Application::StartWakeDetectionIfNeeded() {
//     if (WAKE_WORD_APPLY) {
//         // 原有逻辑：被动监听唤醒词
//         wake_word_->StartDetection();
//     } else {
//         // 新增逻辑：主动进入 Listening 模式
//         // 1. 如果没打开音频通道，先切到 Connecting 并打开它
//         if (!protocol_->IsAudioChannelOpened()) {
//             SetDeviceState(kDeviceStateConnecting);
//             if (!protocol_->OpenAudioChannel()) {
//                 // 通道打开失败，退回 Idle（也可继续唤醒词检测）
//                 SetDeviceState(kDeviceStateIdle);
//                 return;
//             }
//         }
//         // 2. 通道打开后，直接进入 Listening
//         SetListeningMode(kListeningModeAutoStop);
//     }
// }



void Application::CheckNewVersion() {
    const int MAX_RETRY = 10;
    int retry_count = 0;
    int retry_delay = 10; // 初始重试延迟为10秒

    while (true) {
        SetDeviceState(kDeviceStateActivating);
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        if (!ota_.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char buffer[128];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay, ota_.GetCheckVersionUrl().c_str());
            Alert(Lang::Strings::ERROR, buffer, "sad", Lang::Sounds::P3_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay, retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (device_state_ == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2; // 每次重试后延迟时间翻倍
            continue;
        }
        retry_count = 0;
        retry_delay = 10; // 重置重试延迟时间

        if (ota_.HasNewVersion()) {
            Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "happy", Lang::Sounds::P3_UPGRADE);

            vTaskDelay(pdMS_TO_TICKS(3000));

            SetDeviceState(kDeviceStateUpgrading);
            
            display->SetIcon(FONT_AWESOME_DOWNLOAD);
            std::string message = std::string(Lang::Strings::NEW_VERSION) + ota_.GetFirmwareVersion();
            display->SetChatMessage("system", message.c_str());

            auto& board = Board::GetInstance();
            board.SetPowerSaveMode(false);
            wake_word_->StopDetection();
            // 预先关闭音频输出，避免升级过程有音频操作
            auto codec = board.GetAudioCodec();
            codec->EnableInput(false);
            codec->EnableOutput(false);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                audio_decode_queue_.clear();
            }
            background_task_->WaitForCompletion();
            delete background_task_;
            background_task_ = nullptr;
            vTaskDelay(pdMS_TO_TICKS(1000));

            ota_.StartUpgrade([display](int progress, size_t speed) {
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                display->SetChatMessage("system", buffer);
            });

            // If upgrade success, the device will reboot and never reach here
            display->SetStatus(Lang::Strings::UPGRADE_FAILED);
            ESP_LOGI(TAG, "Firmware upgrade failed...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            Reboot();
            return;
        }

        // No new version, mark the current version as valid
        ota_.MarkCurrentVersionValid();
        if (!ota_.HasActivationCode() && !ota_.HasActivationChallenge()) {
            xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota_.HasActivationCode()) {
            ShowActivationCode();
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota_.Activate();
            if (err == ESP_OK) {
                xEventGroupSetBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT);
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (device_state_ == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::ShowActivationCode() {
    auto& message = ota_.GetActivationMessage();
    auto& code = ota_.GetActivationCode();

    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', Lang::Sounds::P3_0},
        digit_sound{'1', Lang::Sounds::P3_1}, 
        digit_sound{'2', Lang::Sounds::P3_2},
        digit_sound{'3', Lang::Sounds::P3_3},
        digit_sound{'4', Lang::Sounds::P3_4},
        digit_sound{'5', Lang::Sounds::P3_5},
        digit_sound{'6', Lang::Sounds::P3_6},
        digit_sound{'7', Lang::Sounds::P3_7},
        digit_sound{'8', Lang::Sounds::P3_8},
        digit_sound{'9', Lang::Sounds::P3_9}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "happy", Lang::Sounds::P3_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status, message, emotion);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        ResetDecoder();
        PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (device_state_ == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}


// 2) 消息处理任务
void Application::MessageProcessingTask(void* pv) {
    auto* app = static_cast<Application*>(pv);
    char* jsonPtr = nullptr;

    while (true) {
        // 阻塞等待，有新 JSON 就拿出来
        if (xQueueReceive(s_jsonQueue, &jsonPtr, portMAX_DELAY) == pdTRUE) {
            // 解析并处理
            app->handleStartTrainingJson(jsonPtr);
            // 处理完记得 free 掉 strdup 出来的内存
            free(jsonPtr);
        }
    }
}

// 3) JSON 解析并分发逻辑
void Application::handleStartTrainingJson(char* json)
{

    cJSON* root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE(TAG, "invalid JSON: %s", json);
        return;
    }
     /* ---------- A. 处理事件 ---------- */
    const cJSON* event = cJSON_GetObjectItemCaseSensitive(root, "event");
    if (cJSON_IsString(event)) {
        if (strcmp(event->valuestring, "setUser") == 0) {
            int uid = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "userId"));

            // 解析可选的 hwWeight（支持 number 或 string）
            float hw = -1.0;
            const cJSON* jhw = cJSON_GetObjectItemCaseSensitive(root, "hwWeight");
            if (cJSON_IsNumber(jhw)) {
                hw = jhw->valuedouble;
            } else if (cJSON_IsString(jhw) && jhw->valuestring) {
                try { hw = std::stof(jhw->valuestring); } catch (...) { hw = -1.0; }
            }

            if (uid > 0) {
                RemoteDataService::GetInstance().SetCurrentUserId(uid);
            

                if (hw > 0.0) {
                    // 将当前配重存到 MCP 层，供后续 create_day 缺省使用
                    McpServer::GetInstance().SetCurrentUserTWeight(hw);
                    ESP_LOGI(TAG, "Bound userId=%d, hwWeight=%.3f", uid, hw);
                } else {
                    ESP_LOGI(TAG, "Bound userId=%d (no valid hwWeight provided)", uid);
                }

                sendToClient(R"({"success":true,"event":"user_bound"})");
                McpServer::GetInstance().SetCurrentUserId(uid);
                // === 新增：把 user_id 写入 NVS（pairing 命名空间）===
                auto& kv = Application::GetInstance().GetPairingSettings();
                kv.SetInt("user_id", uid);
            }
            cJSON_Delete(root);
            return;
        } else if (strcmp(event->valuestring, "skip_rest") == 0) {
            ESP_LOGI(TAG, "Received skip_rest event");
            SkipRest();
            cJSON_Delete(root);
            return;
        } else if (strcmp(event->valuestring, "exit_training") == 0) {
            ESP_LOGI(TAG, "Received exit_training event");
            ExitTraining();
            cJSON_Delete(root);
            return;

        }
    }



    /* ---------- 解析计划 sessionId ---------- */

    const cJSON* sid = cJSON_GetObjectItemCaseSensitive(root, "sessionId");
    if (!cJSON_IsNumber(sid)) sid = cJSON_GetObjectItemCaseSensitive(root, "sid");
    if (!cJSON_IsNumber(sid)) sid = cJSON_GetObjectItemCaseSensitive(root, "planSessionId");
    if (!cJSON_IsNumber(sid)) sid = cJSON_GetObjectItemCaseSensitive(root, "id");

    if (cJSON_IsNumber(sid) && sid->valuedouble > 0) {
        g_plan_session_id = (int)sid->valuedouble;
        ESP_LOGI(TAG, "Plan sessionId cached: %d", g_plan_session_id);
    } else {
        g_plan_session_id = -1;
        ESP_LOGW(TAG, "No valid sessionId in start_training payload");
    }


    /* ---------- B. 处理训练计划 ---------- */
    g_plan_sz = 0;
    g_cur_idx = -1;

    cJSON* items = cJSON_GetObjectItem(root, "items");
    if (items && cJSON_IsArray(items)) {
        const int count = cJSON_GetArraySize(items);
        for (int i = 0; i < count && g_plan_sz < MAX_ITEMS; ++i) {
            cJSON* it = cJSON_GetArrayItem(items, i);
            cJSON* jt = cJSON_GetObjectItem(it, "type");
            cJSON* jr = cJSON_GetObjectItem(it, "reps");
            cJSON* jw = cJSON_GetObjectItem(it, "weight");
            cJSON* jrest = cJSON_GetObjectItem(it, "rest");
            if (!jt || !jr || !jw || !cJSON_IsNumber(jt) || !cJSON_IsNumber(jr) || !cJSON_IsNumber(jw)) {
                ESP_LOGW(TAG, "item[%d] missing fields or not number", i);
                continue;
            }
            int   type   = jt->valueint;
            int   reps   = jr->valueint;
            float weight = (float)cJSON_GetNumberValue(jw);
            int   rest   = (jrest && cJSON_IsNumber(jrest)) ? jrest->valueint : 0;

            if (reps <= 0 || reps > 200)     { ESP_LOGW(TAG, "bad reps=%d", reps); continue; }
            if (weight <= 0 || weight > 200) { ESP_LOGW(TAG, "bad weight=%.2f", weight); continue; }
            if (rest < 0  || rest > 600)     { ESP_LOGW(TAG, "bad rest=%d", rest);  rest = 0; }

            g_plan[g_plan_sz++] = { type, reps, weight, rest };
            ESP_LOGI(TAG, "[%d] type=%d reps=%d weight=%.2f rest=%d", i, type, reps, weight, rest);
        }
    }

    // 计划准备好就启动
    if (g_plan_sz > 0) {
        StartPlan();
    } else {
        ESP_LOGW(TAG, "Empty training plan.");
    }
}


void Application::SkipRest() {

    if (g_in_rest) {
        if (rest_timer_handle) {
            esp_timer_stop(rest_timer_handle);
        }
        g_in_rest = false;
        StartNextItem();
    }
}

void Application::ExitTraining() {
    if (g_in_rest && rest_timer_handle) {
        esp_timer_stop(rest_timer_handle);
        g_in_rest = false;
    }
    is_rep_counting = false;
    g_item_running = false;
    g_cur_idx = g_plan_sz;
    FinishTraining();
}

void Application::PlaySound(const std::string_view& sound) {
    // Wait for the previous sound to finish
    {
        std::unique_lock<std::mutex> lock(mutex_);
        audio_decode_cv_.wait(lock, [this]() {
            return audio_decode_queue_.empty();
        });
    }
    background_task_->WaitForCompletion();

    const char* data = sound.data();
    size_t size = sound.size();
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        AudioStreamPacket packet;
        packet.sample_rate = 16000;
        packet.frame_duration = 60;
        packet.payload.resize(payload_size);
        memcpy(packet.payload.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(packet));
    }
}

void Application::EnterAudioTestingMode() {
    ESP_LOGI(TAG, "Entering audio testing mode");
    ResetDecoder();
    SetDeviceState(kDeviceStateAudioTesting);
}

void Application::ExitAudioTestingMode() {
    ESP_LOGI(TAG, "Exiting audio testing mode");
    SetDeviceState(kDeviceStateWifiConfiguring);
    // Copy audio_testing_queue_ to audio_decode_queue_
    std::lock_guard<std::mutex> lock(mutex_);
    audio_decode_queue_ = std::move(audio_testing_queue_);
    audio_decode_cv_.notify_all();
}

void Application::ToggleChatState() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        EnterAudioTestingMode();
        return;
    } else if (device_state_ == kDeviceStateAudioTesting) {
        ExitAudioTestingMode();
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {
        Schedule([this]() {
            protocol_->CloseAudioChannel();
        });
    }
}

void Application::StartListening() {
    if (device_state_ == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (device_state_ == kDeviceStateWifiConfiguring) {
        EnterAudioTestingMode();
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }
    
    if (device_state_ == kDeviceStateIdle) {
        Schedule([this]() {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    return;
                }
            }

            SetListeningMode(kListeningModeManualStop);
        });
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
            SetListeningMode(kListeningModeManualStop);
        });
    }
}

void Application::StopListening() {
    if (device_state_ == kDeviceStateAudioTesting) {
        ExitAudioTestingMode();
        return;
    }

    const std::array<int, 3> valid_states = {
        kDeviceStateListening,
        kDeviceStateSpeaking,
        kDeviceStateIdle,
    };
    // If not valid, do nothing
    if (std::find(valid_states.begin(), valid_states.end(), device_state_) == valid_states.end()) {
        return;
    }

    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {

    ESP_ERROR_CHECK(nvs_flash_init());
    // NVS 初始化后，再创建 Settings 实例
    pairing_settings_ = std::make_unique<Settings>("pairing", /*read_write=*/true);

    // === 新增：开机即从 NVS 自动绑定上次用户（即使未连接 App 也生效）===
    {
        auto& kv = GetPairingSettings();
        int uid = kv.GetInt("user_id", 0);
        if (uid > 0) {
            RemoteDataService::GetInstance().SetCurrentUserId(uid);
            McpServer::GetInstance().SetCurrentUserId(uid);
           ESP_LOGI(TAG, "Startup auto-bind: uid=%d from NVS", uid);
        } else {
            ESP_LOGI(TAG, "Startup auto-bind: no saved user_id");
        }
    }

     // ─── 在这里新增：创建 JSON 队列 & 启动处理任务 ───
    s_jsonQueue = xQueueCreate(5, sizeof(char*));
    if (!s_jsonQueue) {
        ESP_LOGE(TAG, "Start: failed to create JSON queue");
    } else {
        xTaskCreate(
            MessageProcessingTask,   // 任务入口
            "MsgProcTask",           // 任务名
            4096,                    // 栈大小
            this,                    // 参数传 this 指针
            tskIDLE_PRIORITY + 1,    // 优先级
            nullptr                  // 不要任务句柄
        );
    }
    // ─── 新增结束 ───

    // 在 board.StartNetwork() 之后，不要直接调用 websocket_server_start()
    ESP_LOGI(TAG, "注册 IP_EVENT_STA_GOT_IP 回调");
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        [](void* arg, esp_event_base_t base, int32_t id, void* data) {
            if (!ws_started) {
                ESP_LOGI(TAG, "获取到 IP，启动 WebSocket 服务");
                websocket_server_start();
                ws_started = true;
            }
        },
        nullptr  // 不需要 user_ctx
    ));

    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    if (!imu_initialized_) {
        init_i2c();

        // 打印扫描到的 I2C 地址，调试用
        uint8_t addrs[16];
        uint8_t n = i2c_bus_scan(i2c_bus, addrs, sizeof(addrs));
        ESP_LOGI(TAG, "Found %u I2C device(s):", n);
        for (int i = 0; i < n; ++i) {
            ESP_LOGI(TAG, "  - 0x%02X", addrs[i]);
        }
        init_sensors();
        xTaskCreatePinnedToCore(Application::imu_task,
                        "imu_task",
                        4096,
                        this,
                        4,          // LVGL 默认 5，IMU 设 4 就不会饿死 GUI
                        nullptr,
                        1);         // pin 到 Core 1
                        

        imu_initialized_ = true;


    }



    /* Setup the display */
    auto display = board.GetDisplay();



    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(codec->output_sample_rate(), 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    if (aec_mode_ != kAecOff) {
        ESP_LOGI(TAG, "AEC mode: %d, setting opus encoder complexity to 0", aec_mode_);
        opus_encoder_->SetComplexity(0);
    } else if (board.GetBoardType() == "ml307") {
        ESP_LOGI(TAG, "ML307 board detected, setting opus encoder complexity to 5");
        opus_encoder_->SetComplexity(5);
    } else {
        ESP_LOGI(TAG, "WiFi board detected, setting opus encoder complexity to 0");
        opus_encoder_->SetComplexity(0);
    }

    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    codec->Start();

#if CONFIG_USE_AUDIO_PROCESSOR
    xTaskCreatePinnedToCore([](void* arg) {
        Application* app = (Application*)arg;
        app->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_, 1);
#else
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_);
#endif

    /* Start the clock timer to update the status bar */
    esp_timer_start_periodic(clock_timer_handle_, 1000000);



    /* Wait for the network to be ready */
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);




    //websocket_server_start();               // ★ 启本地 WS 服务器

        /*************** mDNS 初始化 & 注册 ****************/
    {
        /* 1) 初始化 mDNS 模块 */
        ESP_ERROR_CHECK(mdns_init());

        /* 2) 设置主机名（smartdumbbell-XXXX）*/
        std::string mac = SystemInfo::GetMacAddress();          // "AA:BB:CC:DD:EE:FF"
        std::string shortMac = mac.substr(mac.length() - 2*2);  // "EEFF"  末尾 4 个十六进制字符

        char hostname[32];
        snprintf(hostname, sizeof(hostname), "smartdumbbell-%s", shortMac.c_str());
        // → "smartdumbbell-EEFF"
        mdns_hostname_set(hostname);


        /* 3) 设置实例名（手机看到的名称，可自定义）*/
        mdns_instance_name_set("SmartDumbbell");

        /* 4) 向局域网发布一个 UDP 服务，端口 12345 可换成你实际监听的端口 */
        ESP_ERROR_CHECK(mdns_service_add(NULL, "_smartdumbbell", "_tcp", 8080, NULL, 0));


    }

    // Check for new firmware version or get the MQTT broker address
    CheckNewVersion();

    // Initialize the protocol
    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    // Add MCP common tools before initializing the protocol
#if CONFIG_IOT_PROTOCOL_MCP
    McpServer::GetInstance().AddCommonTools();
#endif

    if (ota_.HasMqttConfig()) {
        protocol_ = std::make_unique<MqttProtocol>();
    } else if (ota_.HasWebsocketConfig()) {
        protocol_ = std::make_unique<WebsocketProtocol>();
    } else {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        protocol_ = std::make_unique<MqttProtocol>();
    }

    protocol_->OnNetworkError([this](const std::string& message) {
        SetDeviceState(kDeviceStateIdle);
        Alert(Lang::Strings::ERROR, message.c_str(), "sad", Lang::Sounds::P3_EXCLAMATION);
    });
    protocol_->OnIncomingAudio([this](AudioStreamPacket&& packet) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_state_ == kDeviceStateSpeaking && audio_decode_queue_.size() < MAX_AUDIO_PACKETS_IN_QUEUE) {
            audio_decode_queue_.emplace_back(std::move(packet));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "Server sample rate %d does not match device output sample rate %d, resampling may cause distortion",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
        std::string states;
        if (thing_manager.GetStatesJson(states, false)) {
            protocol_->SendIotStates(states);
        }
#endif
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    background_task_->WaitForCompletion();
                    if (device_state_ == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
#if CONFIG_IOT_PROTOCOL_MCP
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
#endif
#if CONFIG_IOT_PROTOCOL_XIAOZHI
        } else if (strcmp(type->valuestring, "iot") == 0) {
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (cJSON_IsArray(commands)) {
                auto& thing_manager = iot::ThingManager::GetInstance();
                for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
                    auto command = cJSON_GetArrayItem(commands, i);
                    thing_manager.Invoke(command);
                }
            }
#endif
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() {
                        Reboot();
                    });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring, Lang::Sounds::P3_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });
    bool protocol_started = protocol_->Start();

    audio_debugger_ = std::make_unique<AudioDebugger>();
    audio_processor_->Initialize(codec);
    audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                ESP_LOGW(TAG, "Too many audio packets in queue, drop the newest packet");
                return;
            }
        }
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                AudioStreamPacket packet;
                packet.payload = std::move(opus);
#ifdef CONFIG_USE_SERVER_AEC
                {
                    std::lock_guard<std::mutex> lock(timestamp_mutex_);
                    if (!timestamp_queue_.empty()) {
                        packet.timestamp = timestamp_queue_.front();
                        timestamp_queue_.pop_front();
                    } else {
                        packet.timestamp = 0;
                    }

                    if (timestamp_queue_.size() > 3) { // 限制队列长度3
                        timestamp_queue_.pop_front(); // 该包发送前先出队保持队列长度
                        return;
                    }
                }
#endif
                std::lock_guard<std::mutex> lock(mutex_);
                if (audio_send_queue_.size() >= MAX_AUDIO_PACKETS_IN_QUEUE) {
                    ESP_LOGW(TAG, "Too many audio packets in queue, drop the oldest packet");
                    audio_send_queue_.pop_front();
                }
                audio_send_queue_.emplace_back(std::move(packet));
                xEventGroupSetBits(event_group_, SEND_AUDIO_EVENT);
            });
        });
    });
    audio_processor_->OnVadStateChange([this](bool speaking) {
        if (device_state_ == kDeviceStateListening) {
            Schedule([this, speaking]() {
                if (speaking) {
                    voice_detected_ = true;
                } else {
                    voice_detected_ = false;
                }
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            });
        }
    });

    wake_word_->Initialize(codec);
    wake_word_->OnWakeWordDetected([this](const std::string& wake_word) {
        Schedule([this, &wake_word]() {
            if (!protocol_) {
                return;
            }

            if (device_state_ == kDeviceStateIdle) {
                wake_word_->EncodeWakeWordData();

                if (!protocol_->IsAudioChannelOpened()) {
                    SetDeviceState(kDeviceStateConnecting);
                    if (!protocol_->OpenAudioChannel()) {
                        wake_word_->StartDetection();
                        return;
                    }
                }

                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_USE_AFE_WAKE_WORD
                AudioStreamPacket packet;
                // Encode and send the wake word data to the server
                while (wake_word_->GetWakeWordOpus(packet.payload)) {
                    protocol_->SendAudio(packet);
                }
                // Set the chat state to wake word detected
                protocol_->SendWakeWordDetected(wake_word);
#else
                // Play the pop up sound to indicate the wake word is detected
                // And wait 60ms to make sure the queue has been processed by audio task
                ResetDecoder();
                PlaySound(Lang::Sounds::P3_POPUP);
                vTaskDelay(pdMS_TO_TICKS(60));
#endif
                SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
            } else if (device_state_ == kDeviceStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            } else if (device_state_ == kDeviceStateActivating) {
                SetDeviceState(kDeviceStateIdle);
            }
        });
    });
    wake_word_->StartDetection();

    // Wait for the new version check to finish
    xEventGroupWaitBits(event_group_, CHECK_NEW_VERSION_DONE_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
    SetDeviceState(kDeviceStateIdle);

    if (protocol_started) {
        std::string message = std::string(Lang::Strings::VERSION) + ota_.GetCurrentVersion();
        display->ShowNotification(message.c_str());
        display->SetChatMessage("system", "");
        // Play the success sound to indicate the device is ready
        ResetDecoder();
        PlaySound(Lang::Sounds::P3_SUCCESS);
    }

    // Print heap stats
    SystemInfo::PrintHeapStats();
    
    // Enter the main event loop
    MainEventLoop();
}

void Application::OnClockTimer() {
    clock_ticks_++;

    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar();

    // Print the debug info every 10 seconds
    if (clock_ticks_ % 10 == 0) {
        // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
        // SystemInfo::PrintTaskList();
        SystemInfo::PrintHeapStats();

        // If we have synchronized server time, set the status to clock "HH:MM" if the device is idle
        if (ota_.HasServerTime()) {
            if (device_state_ == kDeviceStateIdle) {
                Schedule([this]() {
                    // Set status to clock "HH:MM"
                    time_t now = time(NULL);
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%H:%M  ", localtime(&now));
                    Board::GetInstance().GetDisplay()->SetStatus(time_str);
                });
            }
        }
    }
}

// Add a async task to MainLoop
void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// The Main Event Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainEventLoop() {
    // Raise the priority of the main event loop to avoid being interrupted by background tasks (which has priority 2)
    vTaskPrioritySet(NULL, 3);

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, SCHEDULE_EVENT | SEND_AUDIO_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & SEND_AUDIO_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto packets = std::move(audio_send_queue_);
            lock.unlock();
            for (auto& packet : packets) {
                if (!protocol_->SendAudio(packet)) {
                    break;
                }
            }
        }

        if (bits & SCHEDULE_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

// The Audio Loop is used to input and output audio data
void Application::AudioLoop() {
    auto codec = Board::GetInstance().GetAudioCodec();
    while (true) {
        OnAudioInput();
        if (codec->output_enabled()) {
            OnAudioOutput();
        }
    }
}

void Application::OnAudioOutput() {
    if (busy_decoding_audio_) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
        // Disable the output if there is no audio data for a long time
        if (device_state_ == kDeviceStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                codec->EnableOutput(false);
            }
        }
        return;
    }

    auto packet = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();
    audio_decode_cv_.notify_all();

    // Synchronize the sample rate and frame duration
    SetDecodeSampleRate(packet.sample_rate, packet.frame_duration);

    busy_decoding_audio_ = true;
    background_task_->Schedule([this, codec, packet = std::move(packet)]() mutable {
        busy_decoding_audio_ = false;
        if (aborted_) {
            return;
        }

        std::vector<int16_t> pcm;
        if (!opus_decoder_->Decode(std::move(packet.payload), pcm)) {
            return;
        }
        // Resample if the sample rate is different
        if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(pcm.size());
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), pcm.size(), resampled.data());
            pcm = std::move(resampled);
        }
        codec->OutputData(pcm);
#ifdef CONFIG_USE_SERVER_AEC
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        timestamp_queue_.push_back(packet.timestamp);
#endif
        last_output_time_ = std::chrono::steady_clock::now();
    });
}

void Application::OnAudioInput() {
    if (device_state_ == kDeviceStateAudioTesting) {
        if (audio_testing_queue_.size() >= AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
            ExitAudioTestingMode();
            return;
        }
        std::vector<int16_t> data;
        int samples = OPUS_FRAME_DURATION_MS * 16000 / 1000;
        if (ReadAudio(data, 16000, samples)) {
            background_task_->Schedule([this, data = std::move(data)]() mutable {
                opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                    AudioStreamPacket packet;
                    packet.payload = std::move(opus);
                    packet.frame_duration = OPUS_FRAME_DURATION_MS;
                    packet.sample_rate = 16000;
                    std::lock_guard<std::mutex> lock(mutex_);
                    audio_testing_queue_.push_back(std::move(packet));
                });
            });
            return;
        }
    }

    if (wake_word_->IsDetectionRunning()) {
        std::vector<int16_t> data;
        int samples = wake_word_->GetFeedSize();
        if (samples > 0) {
            if (ReadAudio(data, 16000, samples)) {
                wake_word_->Feed(data);
                return;
            }
        }
    }

    if (audio_processor_->IsRunning()) {
        std::vector<int16_t> data;
        int samples = audio_processor_->GetFeedSize();
        if (samples > 0) {
            if (ReadAudio(data, 16000, samples)) {
                audio_processor_->Feed(data);
                return;
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(OPUS_FRAME_DURATION_MS / 2));
}

bool Application::ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples) {
    auto codec = Board::GetInstance().GetAudioCodec();
    if (!codec->input_enabled()) {
        return false;
    }

    if (codec->input_sample_rate() != sample_rate) {
        data.resize(samples * codec->input_sample_rate() / sample_rate);
        if (!codec->InputData(data)) {
            return false;
        }
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        } else {
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    } else {
        data.resize(samples);
        if (!codec->InputData(data)) {
            return false;
        }
    }
    
    // 音频调试：发送原始音频数据
    if (audio_debugger_) {
        audio_debugger_->Feed(data);
    }
    
    return true;
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    clock_ticks_ = 0;
    auto previous_state = device_state_;
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);
    // The state is changed, wait for all background tasks to finish
    background_task_->WaitForCompletion();

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus(Lang::Strings::STANDBY);
            display->SetEmotion("neutral");
            audio_processor_->Stop();
            wake_word_->StartDetection();
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            timestamp_queue_.clear();
            break;
        case kDeviceStateListening:
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");
            // Update the IoT states before sending the start listening command
#if CONFIG_IOT_PROTOCOL_XIAOZHI
            UpdateIotStates();
#endif

            // Make sure the audio processor is running
            if (!audio_processor_->IsRunning()) {
                // Send the start listening command
                protocol_->SendStartListening(listening_mode_);
                if (previous_state == kDeviceStateSpeaking) {
                    audio_decode_queue_.clear();
                    audio_decode_cv_.notify_all();
                    // FIXME: Wait for the speaker to empty the buffer
                    vTaskDelay(pdMS_TO_TICKS(120));
                }
                opus_encoder_->ResetState();
                audio_processor_->Start();
                wake_word_->StopDetection();
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

            if (listening_mode_ != kListeningModeRealtime) {
                audio_processor_->Stop();
                // Only AFE wake word can be detected in speaking mode
#if CONFIG_USE_AFE_WAKE_WORD
                wake_word_->StartDetection();
#else
                wake_word_->StopDetection();
#endif
            }
            ResetDecoder();
            break;

        case kDeviceStatePowerOff:                       // 🆕 light-sleep
            EnterLightSleep();
            break;
        default:
            // Do nothing
            break;
    }
}

bmi2_dev* Application::GetBmiDev() { return &bmi; }
bmm150_dev* Application::GetBmmDev() { return &bmm; }  //暴露IMU接口


void Application::EnterLightSleep()
{
    ESP_LOGI(TAG, "Preparing for light-sleep…");
    auto& board = Board::GetInstance();
    board.PrepareForLightSleep();

    /* 重新配置 BOOT 键为输入上拉，低电平唤醒 */
    gpio_set_direction(BOOT_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_BUTTON_GPIO, GPIO_PULLUP_ONLY);
    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable(BOOT_BUTTON_GPIO, GPIO_INTR_LOW_LEVEL);

    /* 真正进入 light-sleep —— 立即被 BOOT 低电平唤醒 */
    esp_light_sleep_start();

    /* 醒来后先恢复最基本外设（防闪黑），然后软复位 */
    board.RecoverFromLightSleep();
    ESP_LOGI(TAG, "Woke up from light-sleep, restarting …");
    esp_restart();         
}



void Application::ResetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    audio_decode_cv_.notify_all();
    last_output_time_ = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    codec->EnableOutput(true);
}

void Application::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    if (opus_decoder_->sample_rate() == sample_rate && opus_decoder_->duration_ms() == frame_duration) {
        return;
    }

    opus_decoder_.reset();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(sample_rate, 1, frame_duration);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decoder_->sample_rate(), codec->output_sample_rate());
        output_resampler_.Configure(opus_decoder_->sample_rate(), codec->output_sample_rate());
    }
}

void Application::UpdateIotStates() {
#if CONFIG_IOT_PROTOCOL_XIAOZHI
    auto& thing_manager = iot::ThingManager::GetInstance();
    std::string states;
    if (thing_manager.GetStatesJson(states, true)) {
        protocol_->SendIotStates(states);
    }
#endif
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        Schedule([this]() {
            AbortSpeaking(kAbortReasonNone);
        });
    } else if (device_state_ == kDeviceStateListening) {   
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (device_state_ != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::SendMcpMessage(const std::string& payload) {
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
        case kAecOff:
            audio_processor_->EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
            break;
        case kAecOnServerSide:
            audio_processor_->EnableDeviceAec(false);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        case kAecOnDeviceSide:
            audio_processor_->EnableDeviceAec(true);
            display->ShowNotification(Lang::Strings::RTC_MODE_ON);
            break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::score_poll_task(void*){
    float sc;
    while (true){
        if (fetchScore(sc)){
            RepReport rp1;                    // 根据你已有结构
            rp1.ex       = rep_report.ex;
            rp1.rep_idx  = rep_cnt;           // 最近一次完成的编号
            rp1.score    = sc;                // 服务器返回分
            send_rep_json(rp1);               // 发给 APP
            if (g_cur_idx >= 0 && g_cur_idx < (int)g_record_items.size()) {
                g_record_items[g_cur_idx].scores.push_back(sc);
            }
            ESP_LOGI("FIT", "Score: %.2f, Exercise: %d, Rep Index: %d", sc, rp1.ex, rep_cnt);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



