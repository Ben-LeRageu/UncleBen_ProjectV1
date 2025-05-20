#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"
//#include "driver/touch_sensor.h"
// ATTENTION : Inclusion d’un header interne non documenté.
// Cette fonction peut disparaître ou être modifiée dans une future version d’ESP-IDF.
//#include "esp_private/touch_filter.h"

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (70)// Seuil à 80% de la valeur au repos
#define NUM_TOUCH_BUTTONS 14
#define TOUCH_FILTER_MODE_EN  (1)

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

// Structure des boutons tactiles
typedef struct {
    touch_pad_t pad;
    const char *nom;
} touche_tactile_info_t;

const touche_tactile_info_t boutons_touches[] = {
    { TOUCH_BOUTON_AIRFRY,      "Air Fry" },
    { TOUCH_BOUTON_FRIES,       "Fries" },
    { TOUCH_BOUTON_REHEAT,      "Reheat" },
    { TOUCH_BOUTON_MINUS,       "Minus" },
    { TOUCH_BOUTON_TEMPTIME,    "Temp/Time" },
    { TOUCH_BOUTON_PLUS,        "Plus" },
    { TOUCH_BOUTON_CROQUETTE,   "Croquette" },
    { TOUCH_BOUTON_DEHYDRATE,   "Dehydrate" },
    { TOUCH_BOUTON_KEEPWARM,    "Keep Warm" },
    { TOUCH_BOUTON_PREHEAT,     "Preheat" },
    { TOUCH_BOUTON_STOPCANCEL,  "Stop/Cancel" },
    { TOUCH_BOUTON_POWER,       "Power" },
    { TOUCH_BOUTON_START,       "Start" },
    { TOUCH_BOUTON_TURNREMINDER,"Turn Reminder" },
};

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
    ESP_ERROR_CHECK(touch_pad_init());
for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
{
    ESP_ERROR_CHECK(touch_pad_config(boutons_touches[i].pad));
}
ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));

// Configuration avancée du filtre
    touch_filter_config_t filter_cfg = {
        .mode = TOUCH_PAD_FILTER_IIR_4,   // Mode IIR standard
        .debounce_cnt = 1,                // Déclenche après 1 lecture stable
        .noise_thr = 0,                   // Aucun seuil de bruit (optionnel)
        .jitter_step = 4,                 // Étape de réduction du bruit
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2 // Lissage modéré
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_cfg));
    ESP_ERROR_CHECK(touch_pad_filter_enable());  // Activation du filtre
//ESP_ERROR_CHECK(touch_pad_filter_start(10));

}

// int touch_pad_get_gpio_num(touch_pad_t touch_pad_num)
// {
//     // Mapping manuel basé sur ESP32-S3 datasheet
//     static const int gpio_map[] = {
//         1, 2, 3, 4, 5, 6, 7,
//         8, 9, 10, 11, 12, 13, 14, 15
//     };

//     if (touch_pad_num >= 0 && touch_pad_num < sizeof(gpio_map)/sizeof(gpio_map[0])) {
//         return gpio_map[touch_pad_num];
//     } else {
//         return -1; // Erreur
//     }
// }

void calibrer_thresholds(void)
{
    uint32_t val_filtree = 0;
    for (int i = 0; i < sizeof(boutons_touches)/sizeof(boutons_touches[0]); i++) 
    {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree));
        uint32_t seuil = val_filtree * TOUCH_THRESH_PERCENT /100; // 80% de la valeur au repos
        ESP_ERROR_CHECK(touch_pad_set_thresh(boutons_touches[i].pad, seuil));

        ESP_LOGI("CALIBRATION", "%s: valeur au repos = %"PRIu32" → seuil = %"PRIu32,
                 boutons_touches[i].nom, val_filtree, seuil);
    }
}

void lire_touches_tactiles(void)
{
    uint32_t val_filtree = 0;
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
    {
        touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree);
        ESP_LOGI("TOUCH", "%s (Touch Pad %d) = %"PRIu32,
                 boutons_touches[i].nom,
                 boutons_touches[i].pad,
                 val_filtree);
    }
}
