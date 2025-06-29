#include "bmi270_sensor.h"
#include <esp_log.h>

#define TAG "Bmi270Sensor"

Bmi270Sensor::Bmi270Sensor(i2c_master_bus_handle_t bus, uint8_t addr)
    : I2cDevice(bus, addr)
{
    memset(&dev_, 0, sizeof(dev_));
    dev_.intf_ptr = this;
    dev_.intf = BMI2_I2C_INTF;
    dev_.read = bmi2_i2c_read;
    dev_.write = bmi2_i2c_write;
    dev_.delay_us = bmi2_delay_us;
    dev_.read_write_len = 32;
    dev_.config_file_ptr = NULL;
    dev_.chip_id = 0;
}

bool Bmi270Sensor::Init(uint16_t odr_hz) {
    int8_t rslt = bmi270_init(&dev_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 init fail: %d", rslt);
        return false;
    }
    // 配置加速度计
    memset(&acc_cfg_, 0, sizeof(acc_cfg_));
    acc_cfg_.type = BMI2_ACCEL;
    acc_cfg_.cfg.acc.odr = bmi2_get_odr_enum(odr_hz, 1); // 1=acc
    acc_cfg_.cfg.acc.range = BMI2_ACC_RANGE_8G;
    acc_cfg_.cfg.acc.bwp = BMI2_ACC_BWP_OSR4_AVG1;
    acc_cfg_.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    // 配置陀螺仪
    memset(&gyr_cfg_, 0, sizeof(gyr_cfg_));
    gyr_cfg_.type = BMI2_GYRO;
    gyr_cfg_.cfg.gyr.odr = bmi2_get_odr_enum(odr_hz, 0); // 0=gyr
    gyr_cfg_.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyr_cfg_.cfg.gyr.bwp = BMI2_GYR_BWP_OSR4_MODE;
    gyr_cfg_.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    gyr_cfg_.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    struct bmi2_sens_config cfg[2] = { acc_cfg_, gyr_cfg_ };
    rslt = bmi2_set_sensor_config(cfg, 2, &dev_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 config fail: %d", rslt);
        return false;
    }
    // 启动加速度/陀螺
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &dev_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 enable sensor fail: %d", rslt);
        return false;
    }
    ESP_LOGI(TAG, "BMI270 init OK (ODR=%uHz)", odr_hz);
    return true;
}

bool Bmi270Sensor::ReadRaw(int16_t (&acc)[3], int16_t (&gyr)[3]) {
    struct bmi2_sensor_data data = {0};
    data.type = BMI2_ACCEL | BMI2_GYRO;
    int8_t rslt = bmi2_get_sensor_data(&data, 1, &dev_);
    if (rslt != BMI2_OK) return false;
    acc[0] = data.sens_data.accel.x;
    acc[1] = data.sens_data.accel.y;
    acc[2] = data.sens_data.accel.z;
    gyr[0] = data.sens_data.gyro.x;
    gyr[1] = data.sens_data.gyro.y;
    gyr[2] = data.sens_data.gyro.z;
    return true;
}

uint8_t Bmi270Sensor::WhoAmI() {
    return ReadReg(BMI2_CHIP_ID_ADDR); // 0x00，一般应返回0x24
}
