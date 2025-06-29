#include "bmm150_sensor.h"
#include <esp_log.h>

#define TAG "Bmm150Sensor"

Bmm150Sensor::Bmm150Sensor(i2c_master_bus_handle_t bus, uint8_t addr)
    : I2cDevice(bus, addr)
{
    memset(&dev_, 0, sizeof(dev_));
    dev_.intf_ptr = this;
    dev_.intf = BMM150_I2C_INTF;
    dev_.read = bmm150_i2c_read;
    dev_.write = bmm150_i2c_write;
    dev_.delay_us = bmm150_delay_us;
}

bool Bmm150Sensor::Init() {
    dev_.settings.pwr_mode = BMM150_NORMAL_MODE;
    int8_t rslt = bmm150_init(&dev_);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 init fail: %d", rslt);
        return false;
    }
    // 设置为正常工作模式
    rslt = bmm150_set_op_mode(BMM150_NORMAL_MODE, &dev_);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 set mode fail: %d", rslt);
        return false;
    }
    ESP_LOGI(TAG, "BMM150 init OK");
    return true;
}

bool Bmm150Sensor::ReadRaw(int16_t (&mag)[3]) {
    struct bmm150_mag_data data = {0};
    int8_t rslt = bmm150_read_mag_data(&data, &dev_);
    if (rslt != BMM150_OK) return false;
    mag[0] = data.x;
    mag[1] = data.y;
    mag[2] = data.z;
    return true;
}

uint8_t Bmm150Sensor::WhoAmI() {
    return ReadReg(BMM150_CHIP_ID_ADDR); // 0x40，一般应返回0x32
}
