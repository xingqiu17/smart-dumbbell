// motion_task.cc
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "boards/atoms3r-echo-base/atoms3r_echo_base.h"   // 拿到 Board 单例
#include "application.h"                                  // 发送到云端
#include <stdio.h>

extern "C" void MotionTask(void*) {
    auto& board = AtomS3rEchoBaseBoard::GetInstance();

    int16_t acc[3], gyr[3], mag[3];

    while (true) {
        /* 1. 读取原始数据 */
        if (!board.GetImu()->ReadRaw(acc, gyr)) {
            printf("BMI270 read fail\n");
        }
        if (!board.GetMag()->ReadRaw(mag)) {
            printf("BMM150 read fail\n");
        }

        /* 2. 组装为 JSON 字符串（示例） */
        char payload[128];
        snprintf(payload, sizeof(payload),
                 "{\"acc\":[%d,%d,%d],\"gyr\":[%d,%d,%d],\"mag\":[%d,%d,%d]}",
                 acc[0], acc[1], acc[2],
                 gyr[0], gyr[1], gyr[2],
                 mag[0], mag[1], mag[2]);

        /* 3. 上传——换成你项目里真正的发送接口 */
        Application::GetInstance().SendSensorData(payload);

        /* 4. 控制采样频率：20 ms ≈ 50 Hz */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
