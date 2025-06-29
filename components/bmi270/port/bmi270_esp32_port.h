#pragma once
#include "driver/i2c.h"
#include "bmi2.h"

#ifdef __cplusplus
extern "C" {
#endif

/** I²C 配置（请根据硬件改 GPIO） */
#define BMI270_I2C_PORT      I2C_NUM_0
#define BMI270_I2C_SDA_GPIO  8
#define BMI270_I2C_SCL_GPIO  9
#define BMI270_I2C_FREQ_HZ   400000
#define BMI270_I2C_ADDR      0x68   /* SA0 = 0 -> 0x68，=1 -> 0x69 */

/* 初始化 I²C + bmi2_dev 结构体 */
esp_err_t bmi270_port_init(struct bmi2_dev *dev);

/* 反初始化（可选） */
void bmi270_port_deinit(void);

#ifdef __cplusplus
}
#endif
