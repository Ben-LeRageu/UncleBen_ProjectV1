#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"
#include "freertos/queue.h"

#define TOUCH_THRESH_PERCENT  (80)  // Seuil à 50% de la valeur au repos
#define NUM_TOUCH_BUTTONS     14

static const char *TAG = "TOUCH_MAIN";
static QueueHandle_t queue_touch_evt = NULL;
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS];

// Prototypes
void init_touch_buttons(void);
void calibrer_thresholds(void);
void lire_touches_tactiles(void);
void lire_une_touches_tactile(unsigned char ucNumBouton) ;
void calibrer_touch_seuils(void);
void touch_task(void *param);
static void touch_isr_cb(void *arg);

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

typedef struct {
    uint32_t pad_num;
    uint32_t pad_status;
    touch_pad_intr_mask_t intr_mask;
} touch_evt_t;

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
    if(queue_touch_evt == NULL)
    {
        queue_touch_evt = xQueueCreate(16, sizeof(touch_evt_t));
    }
    init_touch_buttons();
    xTaskCreate(touch_task, "touch_evt_task", 4096, NULL, 5, NULL);
//    ESP_LOGI(TAG, "Init touch ISR");
//     queue_touch_evt = xQueueCreate(10, sizeof(touch_evt_t));

//     touch_pad_init();
//     touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
//     touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

//     for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
//     {
//         touch_pad_config(boutons_touches[i].pad); // Seuil = 0 temporairement
//     }

//     touch_pad_filter_enable();
//     calibrer_touch_seuils();
//     // Activer la fonction de débruitage (utile si bruit environnemental ou câbles longs)
//     touch_pad_denoise_t denoise = {
//         .grade = TOUCH_PAD_DENOISE_BIT4,   // Filtrage niveau moyen (Bit4 = ±8 bits)
//         .cap_level = TOUCH_PAD_DENOISE_CAP_L4,  // Capacité de compensation élevée
//     };
//     ESP_ERROR_CHECK(touch_pad_denoise_set_config(&denoise));
//     ESP_ERROR_CHECK(touch_pad_denoise_enable());
//     ESP_LOGI(TAG, "Denoise activé");

//     touch_pad_isr_register(touch_isr_cb, NULL, TOUCH_PAD_INTR_MASK_ACTIVE);
//     touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE);
//     touch_pad_fsm_start();

//     xTaskCreate(touch_task, "touch_task", 4096, NULL, 5, NULL);
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
     ESP_LOGI(TAG, "Initialisation des boutons tactiles...");
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    for (int i = 0; i < NUM_TOUCH_BUTTONS; ++i) 
    {
        touch_pad_config(boutons_touches[i].pad);
    }
    // Configuration du canal de débruitage (denoise) sur T0
    touch_pad_denoise_t denoise_cfg = {
    .grade = TOUCH_PAD_DENOISE_BIT4,     // 4 bits à annuler
    .cap_level = TOUCH_PAD_DENOISE_CAP_L4 // Niveau de capacité
    };
    ESP_ERROR_CHECK(touch_pad_denoise_set_config(&denoise_cfg));
    ESP_ERROR_CHECK(touch_pad_denoise_enable());
    ESP_LOGI(TAG, "Fonction de débruitage activée");

    touch_filter_config_t filter_cfg = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 1,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_cfg);
    touch_pad_filter_enable();
    ESP_ERROR_CHECK(touch_pad_timeout_set(true, TOUCH_PAD_THRESHOLD_MAX));

    // for (int i = 0; i < NUM_TOUCH_BUTTONS; ++i)
    //  {
    //     touch_pad_read_benchmark(boutons_touches[i].pad, &valeurs_repos[i]);
    //     touch_pad_set_thresh(boutons_touches[i].pad, valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100);
    //     ESP_LOGI("CALIBRATION", "%s: repos = %"PRIu32" → seuil = %"PRIu32,
    //              boutons_touches[i].nom, valeurs_repos[i], valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100);
    // }

    touch_pad_isr_register(touch_isr_cb, NULL, TOUCH_PAD_INTR_MASK_ALL);
    touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT);
    touch_pad_fsm_start();
}

void calibrer_thresholds(void)
{
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
    {
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
//         if (val_filtree < valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100)
// {
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

static void touch_isr_cb(void *arg) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    touch_evt_t evt = {
        .pad_num = touch_pad_get_current_meas_channel(),
        .pad_status = touch_pad_get_status(),
        .intr_mask = touch_pad_read_intr_status_mask(),
    };
    xQueueSendFromISR(queue_touch_evt, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void calibrer_touch_seuils(void)
 {
    for (int i = 0; i < NUM_TOUCH_BUTTONS; i++)
     {
        touch_pad_read_benchmark(boutons_touches[i].pad, &valeurs_repos[i]);
        uint32_t seuil = valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100;
        touch_pad_set_thresh(boutons_touches[i].pad, seuil);

        ESP_LOGI("CALIBRATION", "%s: repos = %"PRIu32" → seuil = %"PRIu32,
                 boutons_touches[i].nom, valeurs_repos[i], seuil);
    }
}

void touch_task(void *param) 
{
    touch_evt_t evt;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    calibrer_touch_seuils();

    while (1) {
        if (xQueueReceive(queue_touch_evt, &evt, portMAX_DELAY))
         {
            for (int i = 0; i < NUM_TOUCH_BUTTONS; ++i) 
            {
                if (boutons_touches[i].pad == evt.pad_num) 
                {
                    if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE)
                     {
                        ESP_LOGI(TAG, "TOUCH DÉTECTÉ: %s (pad %"PRIu32")", boutons_touches[i].nom, evt.pad_num);
                    } 
                    else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) 
                    {
                        ESP_LOGI(TAG, "RELÂCHE: %s (pad %"PRIu32")", boutons_touches[i].nom, evt.pad_num);
                    } 
                    else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) 
                    {
                        ESP_LOGW(TAG, "TIMEOUT sur pad %"PRIu32, evt.pad_num);
                    }
                }
            }
        }
    }
    // touch_evt_t evt;
    // while (1) 
    // {
    //     if (xQueueReceive(queue_touch_evt, &evt, portMAX_DELAY))
    //      {
    //         for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
    //         {
    //             if (boutons_touches[i].pad == evt.pad_num)
    //              {
    //                 ESP_LOGI(TAG, "TOUCH DÉTECTÉ: %s (pad %" PRIu32 ")", boutons_touches[i].nom, evt.pad_num);
    //                 break;
    //             }
    //         }
    //     }
    // }
}