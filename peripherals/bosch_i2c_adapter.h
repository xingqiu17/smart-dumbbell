#pragma once
#include "i2c_device.h"

#ifdef __cplusplus
extern "C" {
#endif

// BMI2/BMI270 适配接口
int8_t bmi2_i2c_read (uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void   bmi2_delay_us(uint32_t period, void *intf_ptr);

// BMM150 适配接口
int8_t bmm150_i2c_read (uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmm150_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void   bmm150_delay_us(uint32_t period, void *intf_ptr);

#ifdef __cplusplus
}
#endif
