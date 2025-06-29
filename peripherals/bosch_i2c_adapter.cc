#include "bosch_i2c_adapter.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring> // for memcpy

// 统一读实现
static int8_t i2c_read_impl(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    if (!intf_ptr) return -1;
    I2cDevice *dev = reinterpret_cast<I2cDevice*>(intf_ptr);
    dev->ReadRegs(reg_addr, data, len);
    return 0; // 0=OK，Bosch标准
}

// 统一写实现
static int8_t i2c_write_impl(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    if (!intf_ptr) return -1;
    I2cDevice *dev = reinterpret_cast<I2cDevice*>(intf_ptr);
    if (len == 1) {
        dev->WriteReg(reg_addr, data[0]);
    } else {
        uint8_t buffer[32]; // 若超32字节可换为 malloc+free 或 std::vector
        if (len + 1 > sizeof(buffer)) return -1; // 太大防御
        buffer[0] = reg_addr;
        memcpy(&buffer[1], data, len);
        // 注意：这里假定I2cDevice有成员i2c_device_
        ESP_ERROR_CHECK(i2c_master_transmit(dev->i2c_device_, buffer, len + 1, 100));
    }
    return 0;
}

// 统一延时实现
static void delay_impl(uint32_t period_us, void*) {
    // period_us 以微秒为单位，FreeRTOS tick最小1ms
    vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000));
}

// BMI2
extern "C" {
int8_t bmi2_i2c_read (uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
    return i2c_read_impl(reg, data, len, intf_ptr);
}
int8_t bmi2_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
    return i2c_write_impl(reg, data, len, intf_ptr);
}
void   bmi2_delay_us(uint32_t period, void *intf_ptr) {
    delay_impl(period, intf_ptr);
}

// BMM150
int8_t bmm150_i2c_read (uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
    return i2c_read_impl(reg, data, len, intf_ptr);
}
int8_t bmm150_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
    return i2c_write_impl(reg, data, len, intf_ptr);
}
void   bmm150_delay_us(uint32_t period, void *intf_ptr) {
    delay_impl(period, intf_ptr);
}
}
