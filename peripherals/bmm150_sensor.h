#pragma once
#include "i2c_device.h"
#include "bosch/bmm150.h"
#include "bosch/bmm150_defs.h"
#include "bosch_i2c_adapter.h"

class Bmm150Sensor : public I2cDevice {
public:
    explicit Bmm150Sensor(i2c_master_bus_handle_t bus, uint8_t addr = 0x10);
    bool Init(); // 默认正常模式
    bool ReadRaw(int16_t (&mag)[3]); // 读取地磁三轴
    uint8_t WhoAmI();

private:
    struct bmm150_dev dev_;
};
