#include "bmm150_esp32_port.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "BMM150_PORT";

/* ---------- 内部 I²C 工具 ---------- */

static int8_t bmm_i2c_read(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t addr = (uint32_t)intf_ptr & 0xFF;
    esp_err_t err = i2c_master_write_read_device(BMM150_I2C_PORT, addr,
                                                 &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
    return err == ESP_OK ? BMM150_OK : BMM150_E_COM_FAIL;
}

static int8_t bmm_i2c_write(uint8_t reg, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t addr = (uint32_t)intf_ptr & 0xFF;

    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    esp_err_t err = i2c_master_write_to_device(BMM150_I2C_PORT, addr,
                                               buf, len + 1, 1000 / portTICK_PERIOD_MS);
    return err == ESP_OK ? BMM150_OK : BMM150_E_COM_FAIL;
}

/* BMM150 使用 ms 级延时 */
static void bmm_delay_ms(uint32_t period, void *intf_ptr)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

/* ---------- 对外 API ---------- */

esp_err_t bmm150_port_init(struct bmm150_dev *dev)
{
    /* 若 BMI270 已经初始化同一 I²C，总线就不用再 install */
    /* 假设在 bmi270_port_init 里 driver_install 已完成 */

    dev->read = bmm_i2c_read;
    dev->write = bmm_i2c_write;
    dev->delay_ms = bmm_delay_ms;
    dev->intf_ptr = (void *)(uint32_t)BMM150_I2C_ADDR;
    dev->intf = BMM150_I2C_INTF;
    return ESP_OK;
}

void bmm150_port_deinit(void)
{
    /* 与 BMI270 共用 I²C，这里一般不删除驱动 */
}
