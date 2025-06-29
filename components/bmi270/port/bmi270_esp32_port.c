#include "bmi270_esp32_port.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "BMI270_PORT";

/* ---------- IDF 原生 I2C 低层 ---------- */

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BMI270_I2C_SDA_GPIO,
        .scl_io_num = BMI270_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BMI270_I2C_FREQ_HZ
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(BMI270_I2C_PORT, &conf), TAG, "param");
    return i2c_driver_install(BMI270_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/* ---------- BMI270 驱动需要的回调 ---------- */

static int8_t bmi_i2c_read(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t dev_addr = (uint32_t)intf_ptr & 0xFF;
    esp_err_t err = i2c_master_write_read_device(BMI270_I2C_PORT, dev_addr,
                                                 &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
    return err == ESP_OK ? BMI2_OK : BMI2_E_COM_FAIL;
}

static int8_t bmi_i2c_write(uint8_t reg, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t dev_addr = (uint32_t)intf_ptr & 0xFF;

    /* 先把寄存器地址拼接到写缓冲前面 */
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    esp_err_t err = i2c_master_write_to_device(BMI270_I2C_PORT, dev_addr,
                                               buf, len + 1, 1000 / portTICK_PERIOD_MS);
    return err == ESP_OK ? BMI2_OK : BMI2_E_COM_FAIL;
}

/* BMI 驱动期望的延时接口，单位 us */
static void bmi_delay_us(uint32_t period, void *intf_ptr)
{
    ets_delay_us(period);
}

/* ---------- 对外 API ---------- */
esp_err_t bmi270_port_init(struct bmi2_dev *dev)
{
    ESP_RETURN_ON_ERROR(i2c_master_init(), TAG, "i2c init");

    dev->read  = bmi_i2c_read;
    dev->write = bmi_i2c_write;
    dev->delay_us = bmi_delay_us;
    dev->intf = BMI2_I2C_INTF;
    dev->intf_ptr = (void *)(uint32_t)BMI270_I2C_ADDR;
    return ESP_OK;
}

void bmi270_port_deinit(void)
{
    i2c_driver_delete(BMI270_I2C_PORT);
}
