#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "esp_mac.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)

static const char *TAG = "TOUCH_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Touch example started");

    touch_pad_init();
    touch_pad_config(TOUCH_PAD_NUM0, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(10);

    while (1) {
        uint16_t touch_value;
        touch_pad_read_filtered(TOUCH_PAD_NUM0, &touch_value);
        ESP_LOGI(TAG, "Touch pad value: %d", touch_value);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
