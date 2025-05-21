#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"

#define TOUCH_THRESH_PERCENT  (60)  // Seuil à 70% de la valeur au repos
#define NUM_TOUCH_BUTTONS     14
#define TAG "TOUCH_TEST"
#define TEST_PAD TOUCH_PAD_NUM10  // Modifier ici si tu veux tester un autre pad

//static const char *TAG = "TOUCH_MAIN";

// Prototypes
void init_touch_buttons(void);
void calibrer_thresholds(void);
void lire_touches_tactiles(void);
void lire_une_touches_tactile(unsigned char ucNumBouton) ;

// Définition des touches
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

// Structure des infos pour chaque bouton
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

// Valeurs au repos
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS] = {0};



void app_main(void)
{
    uint32_t touch_value = 0;

    ESP_LOGI(TAG, "Initialisation du capteur tactile (pad %d)", TEST_PAD);

    // Initialisation du périphérique tactile
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(TEST_PAD)); // pas de seuil

    // Paramètres de tension (standard)
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));

    // FSM en mode timer (déclenchements réguliers)
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));

    // Démarrage manuel si requis
    ESP_ERROR_CHECK(touch_pad_sw_start());

    ESP_LOGI(TAG, "Lecture tactile commencée...");

    while (1) {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(TEST_PAD, &touch_value));
        ESP_LOGI(TAG, "Valeur tactile brute: %lu", touch_value);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // ESP_LOGI(TAG, "Initialisation des boutons tactiles...");
    // init_touch_buttons();
    // vTaskDelay(pdMS_TO_TICKS(100));
    // calibrer_thresholds();
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // while (1) 
    // {
    //     lire_une_touches_tactile(0);
    //     vTaskDelay(pdMS_TO_TICKS(100)); // Assez rapide pour la réactivité, pas trop pour éviter le spam
    // }
}

void init_touch_buttons(void)
{
    ESP_ERROR_CHECK(touch_pad_init());

    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
        ESP_ERROR_CHECK(touch_pad_config(boutons_touches[i].pad));
    }

    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    
    // Configuration du filtre
    touch_filter_config_t filter_cfg = {
        .mode = TOUCH_PAD_FILTER_IIR_4,
        .debounce_cnt = 3,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_cfg));
    ESP_ERROR_CHECK(touch_pad_filter_enable());
    ESP_ERROR_CHECK(touch_pad_sw_start());
}

void calibrer_thresholds(void)
{
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
        uint32_t val_filtree = 0;
        ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree));
        valeurs_repos[i] = val_filtree;
        uint32_t seuil = val_filtree * TOUCH_THRESH_PERCENT / 100;
        ESP_LOGI("CALIBRATION", "%s: repos = %"PRIu32" → seuil = %"PRIu32,
                 boutons_touches[i].nom, val_filtree, seuil);
    }
}

// void lire_touches_tactiles(void)
// {
//     for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) {
//         uint32_t val_filtree = 0;
//         touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree);

//         if (val_filtree < valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100) {
//             ESP_LOGI("TOUCH", "%s activé! (val = %"PRIu32")", 
//                      boutons_touches[i].nom, val_filtree);
//         }
//     }
// }

void lire_touches_tactiles(void)
{
    uint32_t val_filtree = 0;
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
    {
        ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree));

        if (val_filtree < valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100)
        {
            ESP_LOGI("TOUCH", "%s: TOUCHE DÉTECTÉE (val = %"PRIu32", seuil = %"PRIu32")",
                     boutons_touches[i].nom, val_filtree, valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100);
        }
        else
        {
            ESP_LOGI("TOUCH", "%s: rien (val = %"PRIu32")", boutons_touches[i].nom, val_filtree);
        }
    }
}

void lire_une_touches_tactile(unsigned char ucNumBouton)
{
    uint32_t val_filtree = 0;
        ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[ucNumBouton].pad, &val_filtree));

        if (val_filtree < valeurs_repos[ucNumBouton] * TOUCH_THRESH_PERCENT / 100)
        {
            ESP_LOGI("TOUCH", "%s: TOUCHE DÉTECTÉE (val = %"PRIu32", seuil = %"PRIu32")",
                     boutons_touches[ucNumBouton].nom, val_filtree, valeurs_repos[ucNumBouton] * TOUCH_THRESH_PERCENT / 100);
        }
        else
        {
            ESP_LOGI("TOUCH", "%s: rien (val = %"PRIu32")", boutons_touches[ucNumBouton].nom, val_filtree);
        }
}