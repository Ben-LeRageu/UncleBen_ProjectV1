#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t is31fl3731_init(uint8_t addr);
esp_err_t is31fl3731_light_led(uint8_t addr, uint8_t led_index, uint8_t brightness);
