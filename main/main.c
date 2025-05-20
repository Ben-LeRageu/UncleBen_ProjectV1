#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/touch_sensor.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)
#define NUM_TOUCH_BUTTONS 14

// esp_err_t touch_pad_filter_start(uint32_t period_ms);
// esp_err_t touch_pad_filter_read(touch_pad_t pad_num, uint16_t *p_touch_value);
// int touch_pad_get_gpio_num(touch_pad_t touch_pad_num);

void init_touch_buttons(void);
void lire_touches_tactiles(void);
void calibrer_thresholds(void);

static const char *TAG = "TOUCH_MAIN";

typedef enum 
{
    TOUCH_BOUTON_AIRFRY = TOUCH_PAD_NUM10,
    TOUCH_BOUTON_FRIES = TOUCH_PAD_NUM2,
    TOUCH_BOUTON_REHEAT = TOUCH_PAD_NUM3,
    TOUCH_BOUTON_MINUS = TOUCH_PAD_NUM9,
    TOUCH_BOUTON_TEMPTIME = TOUCH_PAD_NUM7,
    TOUCH_BOUTON_PLUS = TOUCH_PAD_NUM8, 
    TOUCH_BOUTON_CROQUETTE = TOUCH_PAD_NUM6,
    TOUCH_BOUTON_DEHYDRATE = TOUCH_PAD_NUM1, 
    TOUCH_BOUTON_KEEPWARM = TOUCH_PAD_NUM5,
    TOUCH_BOUTON_PREHEAT = TOUCH_PAD_NUM14,
    TOUCH_BOUTON_STOPCANCEL = TOUCH_PAD_NUM13,
    TOUCH_BOUTON_POWER = TOUCH_PAD_NUM4,
    TOUCH_BOUTON_START = TOUCH_PAD_NUM12,
    TOUCH_BOUTON_TURNREMINDER = TOUCH_PAD_NUM11
} touch_bouton_t;

// typedef struct {
//     touch_pad_t pad;
//     const char *nom;
// } touche_tactile_info_t;

const char *bouton_noms[] = {
    [TOUCH_BOUTON_AIRFRY] = "AirFry",
    [TOUCH_BOUTON_FRIES] = "Fries",
    [TOUCH_BOUTON_REHEAT] = "Reheat",
    [TOUCH_BOUTON_MINUS] = "Minus",
    [TOUCH_BOUTON_TEMPTIME] = "Temp/Time",
    [TOUCH_BOUTON_PLUS] = "Plus",
    [TOUCH_BOUTON_CROQUETTE] = "Croquette",
    [TOUCH_BOUTON_DEHYDRATE] = "Dehydrate",
    [TOUCH_BOUTON_KEEPWARM] = "KeepWarm",
    [TOUCH_BOUTON_PREHEAT] = "Preheat",
    [TOUCH_BOUTON_STOPCANCEL] = "Stop/Cancel",
    [TOUCH_BOUTON_POWER] = "Power",
    [TOUCH_BOUTON_START] = "Start",
    [TOUCH_BOUTON_TURNREMINDER] = "TurnReminder"
};

static touch_sensor_handle_t controller = NULL;
static touch_pad_t boutons[NUM_TOUCH_BUTTONS] = {
    TOUCH_BOUTON_AIRFRY, TOUCH_BOUTON_FRIES, TOUCH_BOUTON_REHEAT, TOUCH_BOUTON_MINUS,
    TOUCH_BOUTON_TEMPTIME, TOUCH_BOUTON_PLUS, TOUCH_BOUTON_CROQUETTE, TOUCH_BOUTON_DEHYDRATE,
    TOUCH_BOUTON_KEEPWARM, TOUCH_BOUTON_PREHEAT, TOUCH_BOUTON_STOPCANCEL, TOUCH_BOUTON_POWER,
    TOUCH_BOUTON_START, TOUCH_BOUTON_TURNREMINDER
};

static touch_channel_handle_t channels[NUM_TOUCH_BUTTONS];

void app_main(void)
{
    ESP_LOGI(TAG, "Touch example started");

    init_touch_buttons();
    vTaskDelay(pdMS_TO_TICKS(100));
    calibrer_thresholds();

    while (1) 
    {
        lire_touches_tactiles();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void init_touch_buttons(void)
{
    ESP_LOGI(TAG, "Initialisation du controleur de capteurs tactiles");
    
    touch_sensor_controller_config_t ctrl_config = {
        .buffer_type = TOUCH_CHANNEL_IO_BUFFER_TYPE_NO_BUFFER,
        .fsm_mode = TOUCH_FSM_MODE_TIMER,
    };
    ESP_ERROR_CHECK(touch_sensor_install_controller(&ctrl_config, &controller));

    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
        touch_pad_t pad = boutons[i];

        touch_pad_config_t pad_config = {
            .channel_num = pad,
            .channel_type = TOUCH_CHANNEL_TYPE_MEAS,
            .gpio_num = -1 // auto-détection via pad number
        };

        touch_channel_config_t chan_config = {
            .pad_config = pad_config,
            .flags.pullup = true,
        };

        ESP_ERROR_CHECK(touch_sensor_new_channel(controller, &chan_config, &channels[i]));
        ESP_LOGI(TAG, "Canal tactile configuré pour %s (TouchPad %d)", bouton_noms[pad], pad);
    }

    ESP_ERROR_CHECK(touch_sensor_start(controller));
}

int touch_pad_get_gpio_num(touch_pad_t touch_pad_num)
{
    // Mapping manuel basé sur ESP32-S3 datasheet
    static const int gpio_map[] = {
        1, 2, 3, 4, 5, 6, 7,
        8, 9, 10, 11, 12, 13, 14, 15
    };

    if (touch_pad_num >= 0 && touch_pad_num < sizeof(gpio_map)/sizeof(gpio_map[0])) {
        return gpio_map[touch_pad_num];
    } else {
        return -1; // Erreur
    }
}

void lire_touches_tactiles(void)
{
    uint32_t value;
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
        ESP_ERROR_CHECK(touch_channel_read_data(channels[i], &value));
        ESP_LOGI(TAG, "%s = %lu", bouton_noms[boutons[i]], value);
    }
} 


void calibrer_thresholds(void)
{
    uint32_t valeur_repos = 0;
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
        // Lire valeur au repos
        ESP_ERROR_CHECK(touch_channel_read_data(channels[i], &valeur_repos));

        uint32_t seuil = (uint32_t)(valeur_repos * 0.8); // seuil = 80% de repos

        // Appliquer le seuil
        ESP_ERROR_CHECK(touch_channel_set_threshold(channels[i], seuil));

        ESP_LOGI("CALIBRATION", "%s: repos = %lu → seuil = %lu",
                 bouton_noms[boutons[i]], valeur_repos, seuil);
    }
}




