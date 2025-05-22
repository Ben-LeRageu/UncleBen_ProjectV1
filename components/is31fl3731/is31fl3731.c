#include "is31fl3731.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "IS31FL3731"
#define I2C_TIMEOUT_MS 100

esp_err_t is31fl3731_write_register(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t is31fl3731_select_page(uint8_t addr, uint8_t page) {
    return is31fl3731_write_register(addr, 0xFD, page);
}

esp_err_t is31fl3731_init(uint8_t addr) {
    esp_err_t ret = is31fl3731_select_page(addr, 0x0B);
    if (ret != ESP_OK) return ret;
    ret = is31fl3731_write_register(addr, 0x00, 0x01);
    if (ret != ESP_OK) return ret;
    ret = is31fl3731_write_register(addr, 0x01, 0x00);
    ESP_LOGI(TAG, "IS31FL3731 initialized");
    return ret;
}

esp_err_t is31fl3731_light_led(uint8_t addr, uint8_t led_index, uint8_t brightness) {
    if (led_index >= 144) return ESP_ERR_INVALID_ARG;
    is31fl3731_select_page(addr, 0x01);
    return is31fl3731_write_register(addr, led_index, brightness);   
}

void test_all_leds(uint8_t addr) {
    const uint8_t led_indices[] = {
        0, 1, 2, 3, 4, 5,       // CA1 à CA6 sur CB1
        18, 19, 20, 21, 22, 23, // CA1 à CA6 sur CB2
        36, 37, 38, 39, 40, 41  // CA1 à CA6 sur CB3
    };

    if (is31fl3731_init(addr) != ESP_OK) {
        ESP_LOGE("IS31FL3731", "Échec de l'initialisation");
        return;
    }

    for (int i = 0; i < sizeof(led_indices); i++) {
        is31fl3731_light_led(addr, led_indices[i], 255);
    }

    ESP_LOGI("IS31FL3731", "Toutes les LEDs allumées.");
}
