#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>

#include "application.h"
#include "system_info.h"

#define TAG "main"
extern "C" void MotionTask(void*);   // 提前声明

extern "C" void app_main(void)
{
    // Initialize the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS flash for WiFi configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Launch the application
    Application::GetInstance().Start();

    // 启动运动采样任务，固定到 core1，优先级2，2KB栈
    xTaskCreatePinnedToCore(
        MotionTask,           // 任务函数
        "motion",             // 任务名
        2048,                 // 栈空间
        nullptr,              // 参数
        2,                    // 优先级
        nullptr,              // 不关心task handle
        1                     // core1（单核可填 tskNO_AFFINITY）
    );
}
