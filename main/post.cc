// ---- 回调前置声明（无论是否由应用实现，都需要有这一行）----
extern "C" void on_rep_scored_from_server(int rep, int rep_cnt,float score, int label_id, const char* label_name);

// 如果应用侧（application.cc）没有实现，则给一个空的弱实现。
// 应用侧要自定义时：在包含 post.cc 之前  #define SCORER_CB_IMPL_IN_APP  1
#ifndef SCORER_CB_IMPL_IN_APP
extern "C" __attribute__((weak))
void on_rep_scored_from_server(int /*rep*/,int, float /*score*/, int /*label_id*/, const char* /*label_name*/) {
    // 默认空实现
}
#endif

#define AUTH_BEARER_TOKEN   "CHANGE_ME_TOKEN"
/********************  CONFIG – 设备侧 ***************************/
#define RAW_BATCH_SIZE      600
#define FLUSH_INTERVAL_US   1000000000ULL
#define RAW_POST_URL        "http://154.9.24.233:8090/api/imu/raw"
#define FEAT_POST_URL       "http://154.9.24.233:8090/api/imu/feat"
#define SCORE_POST_URL       "http://154.9.24.233:8090/api/score/esp"  



/********************  网络就绪标志 ***************************/
static volatile bool NET_READY = false;
static inline void rawlog_set_network_ready(){ NET_READY = true; }
/********************  依赖 ******************************/
#include "esp_http_client.h"
#include "cJSON.h"
#include <time.h>
#include "esp_log.h"
#include <string.h>
#include <math.h>
// ======== 异步 HTTP 发送器（队列 + 任务）========
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef struct {
    char url[128];
    char* payload;     // 任务里负责 free
    size_t len;
} HttpJob;

static QueueHandle_t s_http_q = nullptr;
static char SESSION_ID[33] = {0};
static inline void ensure_session_id() {
    if (SESSION_ID[0]) return;
    uint32_t r0 = esp_random();
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    uint32_t r3 = esp_random();
    snprintf(SESSION_ID, sizeof(SESSION_ID), "%08lX%08lX%08lX%08lX",
         (unsigned long)r0, (unsigned long)r1, (unsigned long)r2, (unsigned long)r3);
}

struct RawSample {
    int64_t ts;
    float   ax, ay, az;
    float   gx, gy, gz;
    float   qw, qx, qy, qz;
    float   yaw, pitch, roll;
    int     rep;
    uint8_t seg;   // 1=start, 0=middle, 2=end
};

/********************  缓冲 ***************************/
#define PRE_BUFFER_FRAMES   20

static RawSample preBuf[PRE_BUFFER_FRAMES];
static uint8_t   preWr = 0;

static bool      in_segment   = false;
static uint8_t   tail_left    = 0;
static bool      flush_ready  = false;

static RawSample batch[RAW_BATCH_SIZE];
static uint16_t  b_cnt = 0;
static uint64_t  last_flush_us  = 0;

static int g_action_id = 0;  // 由 application.cc 告知当前“动作类型(数字)”
static inline void rawlog_set_action(int action_id){ g_action_id = action_id; }

/********************  小工具：求最值/均值/RMS ***************************/
static inline float fminf3(float a,float b,float c){ return fminf(a,fminf(b,c)); }
static inline float fmaxf3(float a,float b,float c){ return fmaxf(a,fmaxf(b,c)); }

typedef struct { char* data; size_t len; size_t cap; } DynBuf;
static void db_reserve(DynBuf* b, size_t need) {
    if (b->cap >= b->len + need) return;
    size_t ncap = b->cap ? b->cap*2 : 512;
    while (ncap < b->len + need) ncap *= 2;
    b->data = (char*)realloc(b->data, ncap);
    b->cap = ncap;
}
static esp_err_t http_ev_handler(esp_http_client_event_t* evt) {
    if (!evt || !evt->user_data) return ESP_OK;
    DynBuf* db = (DynBuf*)evt->user_data;
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client)) {
            db_reserve(db, evt->data_len + 1);
            memcpy(db->data + db->len, evt->data, evt->data_len);
            db->len += evt->data_len;
            db->data[db->len] = '\0';
        }
        break;
    default: break;
    }
    return ESP_OK;
}

static esp_err_t score_feat_sync(
    int action_id, int rep_id,int rep_cnt,
    const cJSON* feat_obj,     // 你已构好的特征对象
    float* out_score, int* out_label_id, char* out_label, size_t out_label_sz)
{
    if (!NET_READY) return ESP_ERR_INVALID_STATE;

    // 组装请求体：action_id + 扁平特征（把 feat 内的 key 复制到根）
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "action_id", action_id);

    // 将 feat_obj 的所有键值“并到根”
    cJSON* it = nullptr;
    cJSON_ArrayForEach(it, feat_obj) {
        cJSON_AddItemReferenceToObject(root, it->string, it); // 只引用，不拷贝
    }

    char* payload = cJSON_PrintUnformatted(root);
    size_t payload_len = strlen(payload);
    cJSON_Delete(root); // 注意，这会连同feat引用释放，所以 feat_obj 要在外面用“副本”

    DynBuf rx{nullptr,0,0};
    esp_http_client_config_t cfg = {
        .url = SCORE_POST_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 3000,
        .event_handler = http_ev_handler,
        .user_data = &rx
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    esp_http_client_set_header(cli, "Content-Type", "application/json");
    esp_http_client_set_post_field(cli, payload, payload_len);
    esp_err_t err = esp_http_client_perform(cli);
    int code = (err == ESP_OK) ? esp_http_client_get_status_code(cli) : -1;
    esp_http_client_cleanup(cli);
    free(payload);

    if (err != ESP_OK || code != 200 || rx.len == 0) {
        if (rx.data) free(rx.data);
        return ESP_FAIL;
    }

    // 解析响应
    cJSON* resp = cJSON_ParseWithLength(rx.data, rx.len);
    if (!resp) { free(rx.data); return ESP_FAIL; }

    const cJSON* js = cJSON_GetObjectItemCaseSensitive(resp, "score");
    const cJSON* jl = cJSON_GetObjectItemCaseSensitive(resp, "label_id");
    const cJSON* jn = cJSON_GetObjectItemCaseSensitive(resp, "label_name");

    if (cJSON_IsNumber(js) && out_score) *out_score = (float)js->valuedouble;
    if (cJSON_IsNumber(jl) && out_label_id) *out_label_id = jl->valueint;
    if (cJSON_IsString(jn) && out_label && out_label_sz>0) {
        strlcpy(out_label, jn->valuestring, out_label_sz);
    }

    cJSON_Delete(resp);
    free(rx.data);

    // 回调给上层（带 rep 编号）
    if (out_score && out_label_id) {
        on_rep_scored_from_server(rep_id,rep_cnt, *out_score, *out_label_id,
                                  (out_label && out_label[0])? out_label : "");
    }
    return ESP_OK;
}


struct Quat { float w,x,y,z; };

static inline Quat q_conj(Quat q){ return {q.w,-q.x,-q.y,-q.z}; }
static inline Quat q_mul(Quat a, Quat b){
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}
static inline void quat_to_rotvec(Quat q, float out[3]){
    float vn = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z);
    if (vn < 1e-12f) { out[0]=out[1]=out[2]=0.f; return; }
    float ang = 2.f * atan2f(vn, q.w);   // θ
    float s   = ang / vn;                // θ * u = (θ/vn) * v
    out[0] = q.x * s; out[1] = q.y * s; out[2] = q.z * s; // rad
}

// ===== 工具：向量旋转 & 角度解包裹 =====
struct Vec3 { float x,y,z; };

static inline Vec3 rotate_world_to_body(const Quat& q, const Vec3& vw){
    Quat qc = q_conj(q);
    Quat Vw = {0.f, vw.x, vw.y, vw.z};
    Quat t  = q_mul(qc, Vw);
    Quat rb = q_mul(t,   q);         // [0, gx, gy, gz] in body
    return { rb.x, rb.y, rb.z };
}

static inline Vec3 rotate_body_to_world(const Quat& q, const Vec3& vb){
    Quat qc = q_conj(q);
    Quat Vb = {0.f, vb.x, vb.y, vb.z};
    Quat t  = q_mul(q,  Vb);
    Quat rw = q_mul(t,  qc);         // [0, fx, fy, fz] in world
    return { rw.x, rw.y, rw.z };
}

static inline float principal_angle(float a){
    const float PI=3.141592653589793f, TWO_PI=6.283185307179586f;
    while (a >  PI)  a -= TWO_PI;
    while (a <=-PI)  a += TWO_PI;
    return a;
}
static inline float clamp01(float v){ return v>1.f?1.f:(v<-1.f?-1.f:v); }

struct AngleTracker {
    float prev{0}, unwrapped{0}, amin{0}, amax{0};
    bool  init{false};
    void reset(float a0){ prev=a0; unwrapped=0; amin=0; amax=0; init=true; }
    void update(float a_now){
        if(!init){ reset(a_now); return; }
        float d = principal_angle(a_now - prev);
        unwrapped += d; prev = a_now;
        if (unwrapped < amin) amin = unwrapped;
        if (unwrapped > amax) amax = unwrapped;
    }
    float range_deg() const { return (amax - amin) * 57.2957795f; }
};



// 轴向投影：axis=0/1/2 → 设备系 x/y/z
static inline float rom_by_quat_proj(int axis){
    if (b_cnt < 1) return 0.f;
    Quat q0{batch[0].qw, batch[0].qx, batch[0].qy, batch[0].qz};
    float minang=0.f, maxang=0.f;
    for (int i=0;i<b_cnt;++i){
        Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};
        // 相对旋转（从起点到当前）
        Quat dq = q_mul(q_conj(q0), qi);
        // 统一符号，取最短弧
        if (dq.w < 0.f){ dq.w=-dq.w; dq.x=-dq.x; dq.y=-dq.y; dq.z=-dq.z; }
        float rv[3]; quat_to_rotvec(dq, rv); // rad, 在起点的体坐标下
        float ang = rv[axis];                // 主轴上的有符号角
        if (ang < minang) minang = ang;
        if (ang > maxang) maxang = ang;
    }
    return (maxang - minang) * 57.2957795f; // deg
}

static void http_worker(void*){
    // 等待网络就绪
    while (!NET_READY) vTaskDelay(pdMS_TO_TICKS(200));

    for (;;) {
        HttpJob job{};
        if (xQueueReceive(s_http_q, &job, portMAX_DELAY) == pdTRUE) {
            esp_http_client_config_t cfg = {
                .url = job.url,
                .method = HTTP_METHOD_POST,
                .timeout_ms = 3000,
            };
            esp_http_client_handle_t cli = esp_http_client_init(&cfg);
            esp_http_client_set_header(cli, "Content-Type", "application/json");
            esp_http_client_set_post_field(cli, job.payload, job.len);
            esp_err_t err = esp_http_client_perform(cli);
            int code = (err == ESP_OK) ? esp_http_client_get_status_code(cli) : -1;
            esp_http_client_cleanup(cli);

            if (err == ESP_OK) ESP_LOGI("HTTP", "POST %s len=%u OK (%d)", job.url, (unsigned)job.len, code);
            else               ESP_LOGW("HTTP", "POST %s failed: %s", job.url, esp_err_to_name(err));

            free(job.payload);
        }
    }
}

static void http_uploader_init_once(){
    if (s_http_q) return;
    s_http_q = xQueueCreate(8, sizeof(HttpJob));
    configASSERT(s_http_q);
    xTaskCreatePinnedToCore(http_worker, "http_worker", 4096, nullptr, 3, nullptr, 1);
}

static esp_err_t http_post_async(const char* url, const char* json, size_t len){
    if (!s_http_q) http_uploader_init_once();
    HttpJob job{};
    strlcpy(job.url, url, sizeof(job.url));
    job.payload = (char*)malloc(len);     // 任务里 free
    if (!job.payload) return ESP_ERR_NO_MEM;
    memcpy(job.payload, json, len);
    job.len = len;
    return xQueueSend(s_http_q, &job, 0) == pdPASS ? ESP_OK : ESP_FAIL;
}

/********************  HTTP：发特征 ***************************/
/********************  只向打分接口发送特征  ********************/
#define AUTH_BEARER_TOKEN  "CHANGE_ME_TOKEN"                              // TODO: 改成你的Token；未开鉴权可设为空串""

/* 说明：
 * - 依赖：NET_READY / ensure_session_id() / rawlog_set_action(int)
 * - on_rep_scored_from_server(int rep, float score, int label_id, const char* label_name)
 *   需要在 application.cc 里有实现（或在 post.cc 顶部给出弱实现 + 前置声明）
 */

static esp_err_t send_feat_json(int rep_id,int rep_cnt,
    float T_total, float T_conc, float T_ecc,
    float ROM_pitch, float ROM_roll, float ROM_yaw,
    float peak_gyr_main, float smooth_rms, float leak_ratio,
    // —— 新增：线性加速度与通用幅度 —— 
    float ROM_tilt,           // 重力方向改变量（deg）
    float acc_peak_main,      // 主轴线性加速度峰值（m/s^2）
    float acc_rms_main,       // 主轴线性加速度 RMS（m/s^2）
    float acc_jerk_rms,       // 主轴线性加速度 jerk RMS（m/s^3）
    float height_cm)          // 段内竖直位移粗估（cm）
{
    if (!NET_READY) return ESP_ERR_INVALID_STATE;
    ensure_session_id();

    // ---------- 1) 组 features ----------
    cJSON* feat = cJSON_CreateObject();
    if (!feat) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(feat, "T_total", T_total);
    cJSON_AddNumberToObject(feat, "T_conc",  T_conc);
    cJSON_AddNumberToObject(feat, "T_ecc",   T_ecc);
    cJSON_AddNumberToObject(feat, "ROM_pitch", ROM_pitch);
    cJSON_AddNumberToObject(feat, "ROM_roll",  ROM_roll);
    cJSON_AddNumberToObject(feat, "ROM_yaw",   ROM_yaw);
    cJSON_AddNumberToObject(feat, "peak_gyr_main", peak_gyr_main);
    cJSON_AddNumberToObject(feat, "smooth_rms",    smooth_rms);
    cJSON_AddNumberToObject(feat, "leak_ratio",    leak_ratio);
    cJSON_AddNumberToObject(feat, "ROM_tilt",        ROM_tilt);
    cJSON_AddNumberToObject(feat, "acc_peak_main",   acc_peak_main);
    cJSON_AddNumberToObject(feat, "acc_rms_main",    acc_rms_main);
    cJSON_AddNumberToObject(feat, "acc_jerk_rms",    acc_jerk_rms);
    // 你说 height_cm 可忽略，这里不发；要发就解开下一行
    // cJSON_AddNumberToObject(feat, "height_cm",      height_cm);

    // ---------- 2) 组打分请求体 ----------
    // action_id 使用 g_action_id；若未设置，兜底用 1（自己按需改）
    extern int g_action_id;  // 在本文件前部已定义
    int action_id = (g_action_id > 0) ? g_action_id : 1;

    cJSON* req = cJSON_CreateObject();
    if (!req) { cJSON_Delete(feat); return ESP_ERR_NO_MEM; }
    cJSON_AddNumberToObject(req, "action_id", rep_id);
    cJSON_AddItemToObject(req, "features", feat); // 转移所有权：req 删除时一并释放 feat

    char* body = cJSON_PrintUnformatted(req);
    cJSON_Delete(req);
    if (!body) return ESP_ERR_NO_MEM;

    // ---------- 3) 同步 HTTP 调用 /api/score/esp ----------
    DynBuf rx{nullptr,0,0}; // 复用你文件里已有的 DynBuf + http_ev_handler
    esp_http_client_config_t cfg = {
        .url = SCORE_POST_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 3000,
        .event_handler = http_ev_handler,
        .user_data = &rx
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) { free(body); return ESP_FAIL; }

    esp_http_client_set_header(cli, "Content-Type", "application/json");
    if (AUTH_BEARER_TOKEN[0]) {
        char auth[160] = {0};
        snprintf(auth, sizeof(auth), "Bearer %s", AUTH_BEARER_TOKEN);
        esp_http_client_set_header(cli, "Authorization", auth);
    }
    esp_http_client_set_post_field(cli, body, strlen(body));

    esp_err_t err = esp_http_client_perform(cli);
    int status = (err == ESP_OK) ? esp_http_client_get_status_code(cli) : -1;
    esp_http_client_cleanup(cli);
    free(body);

    float score = 0.f;
    int   label_id = -1;
    char  label_name[24] = {0};

    if (err == ESP_OK && status == 200 && rx.len > 0 && rx.data) {
        // 预期响应（/esp）：
        // {"score": 92.3, "error": "NONE"|"ROM_SHORT"|..., "error_code": 0|1|2|...}
        cJSON* resp = cJSON_ParseWithLength(rx.data, rx.len);
        if (resp) {
            cJSON* js = cJSON_GetObjectItemCaseSensitive(resp, "score");
            cJSON* je = cJSON_GetObjectItemCaseSensitive(resp, "error");
            cJSON* jc = cJSON_GetObjectItemCaseSensitive(resp, "error_code");
            if (cJSON_IsNumber(js)) score = (float)js->valuedouble;
            if (cJSON_IsNumber(jc)) label_id = jc->valueint;

            const char* e = (cJSON_IsString(je) && je->valuestring) ? je->valuestring : "NONE";
            if (strcmp(e, "NONE") == 0) {
                strncpy(label_name, "GOOD", sizeof(label_name)-1);
                if (label_id < 0) label_id = 0;
            } else {
                strncpy(label_name, e, sizeof(label_name)-1);
                if (label_id < 0) {
                    // 兜底字符串映射（如果服务端没给 error_code）
                    if      (!strcmp(e,"ROM_SHORT")) label_id = 1;
                    else if (!strcmp(e,"SWING"))     label_id = 2;
                    else if (!strcmp(e,"ASYM"))      label_id = 3;
                    else if (!strcmp(e,"TEMPO"))     label_id = 4;
                    else if (!strcmp(e,"JERK"))      label_id = 5;
                    else                              label_id = 99;
                }
            }
            cJSON_Delete(resp);
        }
        free(rx.data);
    } else {
        if (rx.data) free(rx.data);
        // 失败也回调一次，给上层有机会提示/打点
    }

    // ---------- 4) 回调给上层 ----------
    on_rep_scored_from_server(rep_id,rep_cnt, score, label_id, label_name);

    // 只做打分，不再向 /api/imu/feat 上报
    return ESP_OK;
}




// 机体系三轴索引：按你的设备定义，如需调整只改这三行
#define AX_ROLL   0   // X
#define AX_PITCH  1   // Y（弯举主轴）
#define AX_YAW    2   // Z

static void compute_and_send_features(int rep,int rep_cnt)
{
    if (b_cnt < 4) { b_cnt = 0; flush_ready=false; return; }

    int64_t t0 = batch[0].ts;
    int64_t t1 = batch[b_cnt-1].ts;
    float T_total = (t1 - t0) * 1e-6f;
    if (T_total <= 0) { b_cnt = 0; flush_ready=false; return; }

    // —— 常量/工具 —— 
    const float PI = 3.141592653589793f, TWO_PI = 6.283185307179586f;
    const float RAD2DEG = 57.2957795f, G = 9.80665f;
    const Vec3  gW = {0.f,0.f,1.f};     // 世界重力
    const Vec3  Yb = {0.f,1.f,0.f};     // 机体 Y 轴（如需换轴只改这里）

    auto principal_angle = [&](float a)->float{
        while (a >  PI)  a -= TWO_PI;
        while (a <=-PI)  a += TWO_PI;
        return a;
    };
    auto clamp_m1p1 = [&](float v){ return v>1.f?1.f:(v<-1.f?-1.f:v); };

    // —— 1) 解耦 pitch/roll（fused），以及“顶点”定位 —— 
    float pitch_min =  1e9f, pitch_max = -1e9f;
    float roll_min  =  1e9f, roll_max  = -1e9f;
    int   idx_top   = 0;         // |pitch| 最大点
    float max_abs_pitch = 0.f;

    for (int i=0;i<b_cnt;++i){
        Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};
        Vec3 gB = rotate_world_to_body(qi, gW);
        float pitch_f = asinf( clamp_m1p1(-gB.x) );   // [-pi/2, pi/2]
        float roll_f  = asinf( clamp_m1p1( gB.y) );   // [-pi/2, pi/2]
        if (pitch_f < pitch_min) pitch_min = pitch_f;
        if (pitch_f > pitch_max) pitch_max = pitch_f;
        if (roll_f  < roll_min ) roll_min  = roll_f;
        if (roll_f  > roll_max ) roll_max  = roll_f;

        float ap = fabsf(pitch_f);
        if (ap > max_abs_pitch){ max_abs_pitch = ap; idx_top = i; }
    }
    float ROM_pitch = (pitch_max - pitch_min) * RAD2DEG;
    float ROM_roll  = (roll_max  - roll_min ) * RAD2DEG;

    // —— 2) 相对 yaw（绕重力轴）：退化帧冻结、无解包裹 —— 
    // 选一段“起始稳定向量 h0”（优先 0.15 s 内水平模最大者）
    bool have_h0 = false;
    float h0x=0.f, h0y=0.f;
    int   best_i = -1; float best_n2 = 0.f;
    for (int i=0;i<b_cnt;++i){
        float t = (batch[i].ts - t0) * 1e-6f;
        if (t > 0.15f) break;
        Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};
        Vec3 Yw = rotate_body_to_world(qi, Yb);
        float n2 = Yw.x*Yw.x + Yw.y*Yw.y;
        if (n2 > best_n2){ best_n2 = n2; best_i = i; }
    }
    if (best_i < 0){ best_i = 0; }  // 兜底

    {   // 设定 h0
        Quat qi0{batch[best_i].qw, batch[best_i].qx, batch[best_i].qy, batch[best_i].qz};
        Vec3 Yw0 = rotate_body_to_world(qi0, Yb);
        float n = sqrtf(Yw0.x*Yw0.x + Yw0.y*Yw0.y);
        if (n > 1e-3f){ h0x = Yw0.x/n; h0y = Yw0.y/n; have_h0 = true; }
    }

    float yaw_rel_min = 0.f, yaw_rel_max = 0.f;
    if (have_h0){
        for (int i=0;i<b_cnt;++i){
            Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};
            Vec3 Yw = rotate_body_to_world(qi, Yb);
            float n = sqrtf(Yw.x*Yw.x + Yw.y*Yw.y);
            if (n < 1e-3f) continue;             // 退化：冻结
            float hx = Yw.x/n, hy = Yw.y/n;
            float cross_z = h0x*hy - h0y*hx;     // g·(h0×h)，g=(0,0,1)
            float dot_xy  = h0x*hx + h0y*hy;     // h0·h
            float yaw_rel = atan2f(cross_z, dot_xy); // (-pi,pi]
            if (yaw_rel < yaw_rel_min) yaw_rel_min = yaw_rel;
            if (yaw_rel > yaw_rel_max) yaw_rel_max = yaw_rel;
        }
    }
    float ROM_yaw = (yaw_rel_max - yaw_rel_min) * RAD2DEG;

    // —— 3) 时间分段 —— 
    float T_conc = (batch[idx_top].ts - t0) * 1e-6f;
    float T_ecc  = T_total - T_conc;
    if (T_conc < 0) T_conc = 0;
    if (T_ecc  < 0) T_ecc  = 0;

    // —— 4) 陀螺主轴峰值 & 光滑度（保持原口径） —— 
    float rms_gx=0, rms_gy=0, rms_gz=0;
    for (int i=0;i<b_cnt;++i){
        rms_gx += batch[i].gx * batch[i].gx;
        rms_gy += batch[i].gy * batch[i].gy;
        rms_gz += batch[i].gz * batch[i].gz;
    }
    rms_gx = sqrtf(rms_gx / b_cnt);
    rms_gy = sqrtf(rms_gy / b_cnt);
    rms_gz = sqrtf(rms_gz / b_cnt);

    int main_axis = 0; float rms_main = rms_gx;
    if (rms_gy > rms_main) { main_axis=1; rms_main=rms_gy; }
    if (rms_gz > rms_main) { main_axis=2; rms_main=rms_gz; }

    float peak_gyr_main = 0.0f;
    auto get_g = [&](int i)->float {
        return (main_axis==0)? batch[i].gx : (main_axis==1? batch[i].gy : batch[i].gz);
    };
    for (int i=0;i<b_cnt;++i){
        float av = fabsf(get_g(i));
        if (av > peak_gyr_main) peak_gyr_main = av;
    }

    // —— 5) 串轴泄漏比（基于三轴 ROM） —— 
    float rom_abs[3] = { fabsf(ROM_roll), fabsf(ROM_pitch), fabsf(ROM_yaw) };
    int   rom_main_i = 0;
    if (rom_abs[1] > rom_abs[rom_main_i]) rom_main_i = 1;
    if (rom_abs[2] > rom_abs[rom_main_i]) rom_main_i = 2;
    float rom_main = rom_abs[rom_main_i];
    float leak_ratio = 0.0f;
    if (rom_main > 1e-6f) {
        float leak = rom_abs[0] + rom_abs[1] + rom_abs[2] - rom_main;
        leak_ratio = leak / rom_main;
    }

    // —— 6) 线性加速度（去重力）、ROM_tilt、height_cm（改为“幅度”） —— 
    // 6.1 ROM_tilt：重力方向相对段首的最大夹角
    Quat q0{batch[0].qw, batch[0].qx, batch[0].qy, batch[0].qz};
    // 注意：上行最后一个成员应为 q0.z（修正：下面重新赋值确保正确）
    q0 = {batch[0].qw, batch[0].qx, batch[0].qy, batch[0].qz};
    Vec3 gB0 = rotate_world_to_body(q0, gW);
    float tilt_max_rad = 0.f;

    // 6.2 主轴线性加速度峰值/RMS + jerk RMS（在“线性加速度主轴”上算）
    float acc_rms_x=0, acc_rms_y=0, acc_rms_z=0;
    float acc_peak_x=0, acc_peak_y=0, acc_peak_z=0;

    // 6.3 世界竖直加速度数组（用于位移），时间轴
    static float t_arr[RAW_BATCH_SIZE];
    static float azW_arr[RAW_BATCH_SIZE];
    static float v_arr[RAW_BATCH_SIZE];
    t_arr[0] = 0.f;

    for (int i=0;i<b_cnt;++i){
        Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};

        // 机体系重力方向（用于 tilt 与去重力）
        Vec3 gB = rotate_world_to_body(qi, gW);

        // —— ROM_tilt —— 
        float c = clamp_m1p1( gB.x*gB0.x + gB.y*gB0.y + gB.z*gB0.z );
        float tilt = acosf(c);
        if (tilt > tilt_max_rad) tilt_max_rad = tilt;

        // —— 线性加速度（机体）：a_lin_b = a_b - g_b —— 
        float ax_ms2 = batch[i].ax * G;
        float ay_ms2 = batch[i].ay * G;
        float az_ms2 = batch[i].az * G;
        float ax_lin = ax_ms2 - gB.x*G;
        float ay_lin = ay_ms2 - gB.y*G;
        float az_lin = az_ms2 - gB.z*G;

        // 主轴 RMS/峰值（机体系）
        acc_rms_x += ax_lin*ax_lin;
        acc_rms_y += ay_lin*ay_lin;
        acc_rms_z += az_lin*az_lin;
        acc_peak_x = fmaxf(acc_peak_x, fabsf(ax_lin));
        acc_peak_y = fmaxf(acc_peak_y, fabsf(ay_lin));
        acc_peak_z = fmaxf(acc_peak_z, fabsf(az_lin));

        // —— 位移：把“原始加速度”旋到世界再减重力，数值更稳 —— 
        Vec3 aW_raw = rotate_body_to_world(qi, {ax_ms2, ay_ms2, az_ms2});
        azW_arr[i] = aW_raw.z - G;     // 世界竖直线加速度

        if (i>0){
            float dt = (batch[i].ts - batch[i-1].ts) * 1e-6f;
            t_arr[i] = t_arr[i-1] + ((dt>0.f && dt<0.05f)? dt : 0.f);
        }
    }

    // 线性加速度主轴选择 + RMS/峰值
    acc_rms_x = sqrtf(acc_rms_x / b_cnt);
    acc_rms_y = sqrtf(acc_rms_y / b_cnt);
    acc_rms_z = sqrtf(acc_rms_z / b_cnt);
    float acc_rms_arr[3]  = {acc_rms_x, acc_rms_y, acc_rms_z};
    float acc_peak_arr[3] = {acc_peak_x, acc_peak_y, acc_peak_z};
    int   acc_main_i = (acc_rms_arr[1] > acc_rms_arr[0]) ? 1 : 0;
    if (acc_rms_arr[2] > acc_rms_arr[acc_main_i]) acc_main_i = 2;
    float acc_rms_main  = acc_rms_arr[acc_main_i];
    float acc_peak_main = acc_peak_arr[acc_main_i];

    // 线性加速度 jerk RMS（沿线性加速度主轴）
    auto acc_lin_of = [&](int i)->float{
        Quat qi{batch[i].qw, batch[i].qx, batch[i].qy, batch[i].qz};
        Vec3 gB = rotate_world_to_body(qi, gW);
        float ax_ms2 = batch[i].ax*G, ay_ms2 = batch[i].ay*G, az_ms2 = batch[i].az*G;
        float ax_lin = ax_ms2 - gB.x*G;
        float ay_lin = ay_ms2 - gB.y*G;
        float az_lin = az_ms2 - gB.z*G;
        return (acc_main_i==0)? ax_lin : (acc_main_i==1? ay_lin : az_lin);
    };
    float acc_jerk_sum=0.f; int acc_jerk_n=0;
    for (int i=1;i<b_cnt;++i){
        float dt = (batch[i].ts - batch[i-1].ts) * 1e-6f;
        if (dt<=0.f || dt>0.05f) continue;
        float jerk = (acc_lin_of(i) - acc_lin_of(i-1)) / dt;   // m/s^3
        acc_jerk_sum += jerk*jerk; acc_jerk_n++;
    }
    float acc_jerk_rms = (acc_jerk_n>0)? sqrtf(acc_jerk_sum/acc_jerk_n) : 0.f;

    // —— 6.4 height_cm：偏置估计 + 速度漂移消除 + 位移幅度（max−min） —— 
    // 静止窗：段首/段末 0.25s 内，|a|≈g 且 |gyr| 小
    auto near_static = [&](int i)->bool{
        float gmag = sqrtf(batch[i].ax*batch[i].ax + batch[i].ay*batch[i].ay + batch[i].az*batch[i].az);
        float gyr  = sqrtf(batch[i].gx*batch[i].gx + batch[i].gy*batch[i].gy + batch[i].gz*batch[i].gz);
        return (fabsf(gmag - 1.0f) < 0.05f) && (gyr < 0.35f);  // 0.35 rad/s ≈ 20 dps
    };
    float T = t_arr[b_cnt-1];
    float bias = 0.f; int n_bias = 0;
    for (int i=0;i<b_cnt;++i){
        if (t_arr[i] < 0.25f && near_static(i)) { bias += azW_arr[i]; n_bias++; }
        if (T - t_arr[i] < 0.25f && near_static(i)) { bias += azW_arr[i]; n_bias++; }
    }
    if (n_bias>0) bias /= n_bias;
    for (int i=0;i<b_cnt;++i) azW_arr[i] -= bias;

    // 加速度→速度（梯形积分）
    v_arr[0] = 0.f;
    for (int i=1;i<b_cnt;++i){
        float dt = t_arr[i] - t_arr[i-1];
        if (dt<=0.f) { v_arr[i] = v_arr[i-1]; continue; }
        v_arr[i] = v_arr[i-1] + 0.5f*(azW_arr[i-1]+azW_arr[i]) * dt;
    }
    // 线性漂移消除：强制段末速度为0
    float v_end = v_arr[b_cnt-1];
    float z=0.f, z_min=0.f, z_max=0.f, v_prev_corr = 0.f;
    for (int i=1;i<b_cnt;++i){
        float dt = t_arr[i] - t_arr[i-1];
        float frac = (T>0.f)? (t_arr[i]/T) : 0.f;
        float v_corr = v_arr[i] - frac * v_end;
        z += 0.5f * (v_prev_corr + v_corr) * dt;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
        v_prev_corr = v_corr;
    }
    float height_cm = (z_max - z_min) * 100.f;

    // —— 7) jerk（陀螺主轴角速度一阶差分 RMS） —— 
    float smooth_acc = 0.0f; int smooth_n = 0;
    for (int i=1;i<b_cnt;++i){
        float dt = (batch[i].ts - batch[i-1].ts) * 1e-6f;
        if (dt <= 0.f || dt > 0.2f) continue;
        float jerk = (get_g(i) - get_g(i-1)) / dt;   // rad/s^2
        smooth_acc += jerk * jerk; smooth_n++;
    }
    float smooth_rms = (smooth_n > 0)? sqrtf(smooth_acc / smooth_n) : 0.0f;

    // —— 8) 上报 —— 
    int rep_id = rep;
    send_feat_json(rep_id,rep_cnt, T_total, T_conc, T_ecc,
                   ROM_pitch, ROM_roll, ROM_yaw,
                   peak_gyr_main, smooth_rms, leak_ratio,
                   tilt_max_rad*RAD2DEG, acc_peak_main, acc_rms_main, acc_jerk_rms, height_cm);

    b_cnt = 0;
    flush_ready = false;
}


static inline void flush_batch_if_needed()
{
    uint64_t now = esp_timer_get_time();
    if (b_cnt == 0) return;
    if (b_cnt >= RAW_BATCH_SIZE ) {
        // 训练采集模式才用；上线默认不开
        // send_raw_batch(batch, b_cnt);
        b_cnt = 0;
        last_flush_us = now;
    }
}

static inline void append_to_batch(const RawSample &s){
    if (b_cnt < RAW_BATCH_SIZE) {
        batch[b_cnt++] = s;
    } else {
        flush_batch_if_needed();
        batch[b_cnt++] = s;
    }
}

/********************  对外 API  *********************************/
static inline void rawlog_init() {
    if (last_flush_us == 0) last_flush_us = esp_timer_get_time();
}

static inline void push_sample(
        int rep_id,int64_t ts_us,
        float ax,float ay,float az,
        float gx,float gy,float gz,
        float qw,float qx,float qy,float qz,
        float yaw,float pitch,float roll)
{
    RawSample s{ts_us,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,yaw,pitch,roll,rep_id,0};

    if (in_segment) {
        append_to_batch(s);
    }
    else{
        preBuf[preWr] = s;
        preWr = (preWr + 1) % PRE_BUFFER_FRAMES;
    }
}

static inline void segment_start(){
    if (in_segment) return;
    in_segment = true;
    b_cnt = 0;

    for (uint8_t i = 0; i < preWr; ++i) {
        RawSample s = preBuf[i];
        s.seg = (i == 0 ? 1 : 0);
        append_to_batch(s);
    }
    preWr = 0;
}

static inline void segment_end(int rep,int rep_cnt){
    if (!in_segment) return;
    in_segment = false;
    compute_and_send_features(rep,rep_cnt);
}


