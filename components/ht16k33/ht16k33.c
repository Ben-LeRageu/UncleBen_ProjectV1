#include "ht16k33.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "HT16K33"
#define I2C_TIMEOUT_MS 100

esp_err_t ht16k33_write_cmd(uint8_t addr, uint8_t cmd) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

esp_err_t ht16k33_init(uint8_t addr) {
    esp_err_t ret;
    ret = ht16k33_write_cmd(addr, 0x21);
    if (ret != ESP_OK) return ret;
    ret = ht16k33_write_cmd(addr, 0x81);
    if (ret != ESP_OK) return ret;
    ret = ht16k33_write_cmd(addr, 0xEF);
    return ret;
}
