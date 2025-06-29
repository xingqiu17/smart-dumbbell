#pragma once
#include "driver/i2c.h"
#include "bmm150.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMM150_I2C_PORT      I2C_NUM_0
#define BMM150_I2C_SDA_GPIO  8   /* 与 BMI270 复用一条总线 */
#define BMM150_I2C_SCL_GPIO  9
#define BMM150_I2C_FREQ_HZ   400000
#define BMM150_I2C_ADDR      0x10

esp_err_t bmm150_port_init(struct bmm150_dev *dev);
void      bmm150_port_deinit(void);

#ifdef __cplusplus
}
#endif
