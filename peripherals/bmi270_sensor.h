#pragma once
#include "i2c_device.h"
#include "bosch/bmi270.h"
#include "bosch/bmi2_defs.h"
#include "bosch_i2c_adapter.h"

// BMI270高层传感器类，仅暴露最常用API
class Bmi270Sensor : public I2cDevice {
public:
    explicit Bmi270Sensor(i2c_master_bus_handle_t bus, uint8_t addr = 0x68);
    bool Init(uint16_t odr_hz = 100); // odr_hz 10~800, 一般用100~200
    bool ReadRaw(int16_t (&acc)[3], int16_t (&gyr)[3]); // 单次读取
    uint8_t WhoAmI();

    // 可按需添加更多BMI2/BMI270功能封装

private:
    struct bmi2_dev dev_;
    struct bmi2_sens_config acc_cfg_, gyr_cfg_;
};
