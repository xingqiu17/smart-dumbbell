#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <list>
#include <vector>
#include <condition_variable>
#include <memory>

#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>

#include "protocol.h"
#include "ota.h"
#include "background_task.h"
#include "audio_processor.h"
#include "wake_word.h"
#include "audio_debugger.h"
#include "settings.h"
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SCHEDULE_EVENT (1 << 0)
#define SEND_AUDIO_EVENT (1 << 1)
#define CHECK_NEW_VERSION_DONE_EVENT (1 << 2)

enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

enum DeviceState {
    kDeviceStateUnknown,
    kDeviceStateStarting,
    kDeviceStateWifiConfiguring,
    kDeviceStateIdle,
    kDeviceStateConnecting,
    kDeviceStateListening,
    kDeviceStateSpeaking,
    kDeviceStateUpgrading,
    kDeviceStateActivating,
    kDeviceStateAudioTesting,
    kDeviceStateFatalError,
    kDeviceStatePowerOff   // 设备关机状态
};

#define OPUS_FRAME_DURATION_MS 60
#define MAX_AUDIO_PACKETS_IN_QUEUE (2400 / OPUS_FRAME_DURATION_MS)
#define AUDIO_TESTING_MAX_DURATION_MS 10000





struct bmi2_sens_data; 
struct bmi2_dev;
struct bmm150_dev;

/* —— 简易动作分类 —— */
enum Exercise {
    EX_UNKNOWN = 0,
    EX_AIDBC   = 2,
    EX_30DBP   = 3,
    EX_DWC     = 1,
    EX_DLR     = 4,
    EX_45DBP   = 5,
    EX_DSP     = 6,
    EX_IDBC    = 7
};

struct RepReport {
    Exercise ex;
    uint32_t rep_idx;      // 从 1 开始
    float    score;        // 0-100
};

class Application {
public:

    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;


    static Application& GetInstance();
    static bmi2_dev* GetBmiDev();
    static bmm150_dev* GetBmmDev();
    bool GetLatestImu(bmi2_sens_data& out);/** 取出“最近一次”IMU 原始数据（若队列为空返回 false） */
    static bool GetLatestMag(float out[3]);
    void Start();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return voice_detected_; }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void UpdateIotStates();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    void PlaySound(const std::string_view& sound);
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    AecMode GetAecMode() const { return aec_mode_; }
    BackgroundTask* GetBackgroundTask() const { return background_task_; }
    Settings& GetPairingSettings() { return *pairing_settings_;}
    /* ────────────────────────── 新增 ────────────────────────── */
    /** JSON 消息队列，存 strdup() 出来的 char* */
    static QueueHandle_t s_jsonQueue;

    /** ws_handler 会调用，把接收到的 JSON char* 推入这里 */
    static QueueHandle_t GetJsonQueue();

    /** 启动后台消费任务时的入口函数 */
    static void MessageProcessingTask(void* pv);

    /** 真正解析并分发 start_training JSON 的函数 */
    void handleStartTrainingJson(char* json);
    /* ────────────────────────── 新增 End ────────────────────────── */


    

    

private:
    Application();
    ~Application();
 

    std::unique_ptr<WakeWord> wake_word_;
    std::unique_ptr<AudioProcessor> audio_processor_;
    std::unique_ptr<AudioDebugger> audio_debugger_;
    Ota ota_;
    std::mutex mutex_;
    std::list<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;
    static QueueHandle_t  s_imuQueue;   ///< 环形队列句柄
    static QueueHandle_t  s_magQueue;   ///< 环形队列句柄
    std::unique_ptr<Settings> pairing_settings_;   // 新增：配对用 NVS




    bool imu_initialized_ = false;
    bool aborted_ = false;
    bool voice_detected_ = false;
    bool busy_decoding_audio_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;

    // Audio encode / decode
    TaskHandle_t audio_loop_task_handle_ = nullptr;
    BackgroundTask* background_task_ = nullptr;
    std::chrono::steady_clock::time_point last_output_time_;
    std::list<AudioStreamPacket> audio_send_queue_;
    std::list<AudioStreamPacket> audio_decode_queue_;
    std::condition_variable audio_decode_cv_;
    std::list<AudioStreamPacket> audio_testing_queue_;

    // 新增：用于维护音频包的timestamp队列
    std::list<uint32_t> timestamp_queue_;
    std::mutex timestamp_mutex_;

    std::unique_ptr<OpusEncoderWrapper> opus_encoder_;
    std::unique_ptr<OpusDecoderWrapper> opus_decoder_;

    OpusResampler input_resampler_;
    OpusResampler reference_resampler_;
    OpusResampler output_resampler_;

        
    

    void StartWakeDetectionIfNeeded();
    static void init_i2c();
    static void init_sensors();
    static void imu_task(void*);
    void MainEventLoop();
    void OnAudioInput();
    void OnAudioOutput();
    bool ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples);
    void ResetDecoder();
    void SetDecodeSampleRate(int sample_rate, int frame_duration);
    void CheckNewVersion();
    void ShowActivationCode();
    void OnClockTimer();
    void SetListeningMode(ListeningMode mode);
    void AudioLoop();
    void EnterAudioTestingMode();
    void ExitAudioTestingMode();
    // void EnterDeepSleep();  // 进入深度睡眠模式
    void EnterLightSleep();  // 进入浅睡眠模式
    void classify_and_count();
    static float score_rep(Exercise type, float dP, float dR, uint64_t dt_us);
    static Exercise classify_rep(const float v_base[3],float dPitch, float dRoll,float rms_omega);
};

#endif // _APPLICATION_H_
