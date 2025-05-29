#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t is31fl3731_init(uint8_t addr);
esp_err_t is31fl3731_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t is31fl3731_select_page(uint8_t addr, uint8_t page);
esp_err_t is31fl3731_light_ledCA(uint8_t addr, uint8_t indiceCA, uint8_t etatCA);
esp_err_t is31fl3731_light_ledCB(uint8_t addr, uint8_t indiceCB, uint8_t etatCB);
