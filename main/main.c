#include <stdio.h>
#include <inttypes.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"
#include "freertos/queue.h"
//#include "is31fl3731.h"

#define I2C_TIMEOUT_MS 100
#define I2C_MASTER_SCL_IO  GPIO_NUM_21  // adapte selon ton schéma
#define I2C_MASTER_SDA_IO  GPIO_NUM_20  // adapte selon ton schéma
#define I2C_MASTER_NUM     I2C_NUM_0    // I2C port
#define I2C_MASTER_FREQ_HZ 400000       // 400kHz pour IS31FL3731
#define I2C_MASTER_TX_BUF_DISABLE 0     // Pas de buffer en mode master
#define I2C_MASTER_RX_BUF_DISABLE 0

void test_all_leds(uint8_t addr);
esp_err_t is31fl3731_init(uint8_t addr);
esp_err_t is31fl3731_select_page(uint8_t addr, uint8_t page);
//esp_err_t is31fl3731_write_register(uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t is31fl3731_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t is31fl3731_light_led(uint8_t addr, uint8_t led_index, uint8_t brightness);
esp_err_t i2c_master_init(void);

#define TOUCH_THRESH_PERCENT  (2)  // Seuil à 2% de la valeur au repos
#define NUM_TOUCH_BUTTONS     14

static const char *TAG = "TOUCH_MAIN";
static QueueHandle_t queue_touch_evt = NULL;
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS];

// Prototypes
void init_touch_buttons(void);
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
    TOUCH_BOUTON_NUGGETS = TOUCH_PAD_NUM6,
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
    { TOUCH_BOUTON_NUGGETS,     "Nuggets" },
    { TOUCH_BOUTON_DEHYDRATE,   "Dehydrate" },
    { TOUCH_BOUTON_KEEPWARM,    "Keep Warm" },
    { TOUCH_BOUTON_PREHEAT,     "Preheat" },
    { TOUCH_BOUTON_STOPCANCEL,  "Stop/Cancel" },
    { TOUCH_BOUTON_POWER,       "Power" },
    { TOUCH_BOUTON_START,       "Start" },
    { TOUCH_BOUTON_TURNREMINDER,"Turn Reminder" },
};

typedef struct {
    int temp;             // Température (ex: 400)
    char time[6];         // Temps au format "MM:SS", ex: "12:30"
    char state[16];       // "cooking", "paused", "stopped"
    char mode[4];         // "AP" ou "STA"
} air_fryer_status_t;

// Valeurs au repos
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS] = {0};



void app_main(void)
{
    uint8_t addr = 0x74;
    air_fryer_status_t fryer = {
     .temp = 0,
     .time = "00:00",
     .state = "OFF",
     .mode = "AP"
    };
    ESP_ERROR_CHECK(i2c_master_init());
    //is31fl3731_init(addr);
    if (is31fl3731_init(addr) != ESP_OK) {
        ESP_LOGE("IS31FL3731", "Échec de l'initialisation");
        return;
    }
   // test_all_leds(addr);
    //airfryer_set_etat(&fryer);
    if(queue_touch_evt == NULL)
    {
        queue_touch_evt = xQueueCreate(16, sizeof(touch_evt_t));
    }
    init_touch_buttons();
    xTaskCreate(touch_task, "touch_evt_task", 4096, NULL, 5, NULL);
}

//fonction init I2C
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t err;

    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    return err;
}

void init_touch_buttons(void)
{
    //static uint32_t denoise_value = 0 ;
     ESP_LOGI(TAG, "Initialisation des boutons tactiles...");
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    for (int i = 0; i < NUM_TOUCH_BUTTONS; ++i) 
    {
        touch_pad_config(boutons_touches[i].pad);
    }
    // touch_pad_denoise_read_data(*denoise_value);
    // ESP_LOGI("valeur de ref de denoise:", ,denoise_value);
    // Configuration du canal de débruitage (denoise) sur T0
    touch_pad_denoise_t denoise_cfg = {
    .grade = TOUCH_PAD_DENOISE_BIT4,     // 4 bits à annuler
    .cap_level = TOUCH_PAD_DENOISE_CAP_L0 // Niveau de capacité
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

    touch_pad_isr_register(touch_isr_cb, NULL, TOUCH_PAD_INTR_MASK_ALL);
    touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT);
    touch_pad_fsm_start();
}

// void lire_touches_tactiles(void)
// {
//     uint32_t val_filtree = 0;
//     for (int i = 0; i < NUM_TOUCH_BUTTONS; i++) 
//     {
//         ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[i].pad, &val_filtree));
//         if (val_filtree < valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100)
//         {
//             ESP_LOGI("TOUCH", "%s: TOUCHE DÉTECTÉE (val = %"PRIu32", seuil = %"PRIu32")",
//                      boutons_touches[i].nom, val_filtree, valeurs_repos[i] * TOUCH_THRESH_PERCENT / 100);
//         }
//         else
//         {
//             ESP_LOGI("TOUCH", "%s: rien (val = %"PRIu32")", boutons_touches[i].nom, val_filtree);
//         }
//     }
// }

// void lire_une_touches_tactile(unsigned char ucNumBouton)
// {
//     uint32_t val_filtree = 0;
//         ESP_ERROR_CHECK(touch_pad_read_raw_data(boutons_touches[ucNumBouton].pad, &val_filtree));
//         if (val_filtree < valeurs_repos[ucNumBouton] * TOUCH_THRESH_PERCENT / 100)
//         {
//             ESP_LOGI("TOUCH", "%s: TOUCHE DÉTECTÉE (val = %"PRIu32", seuil = %"PRIu32")",
//                      boutons_touches[ucNumBouton].nom, val_filtree, valeurs_repos[ucNumBouton] * TOUCH_THRESH_PERCENT / 100);
//         }
//         else
//         {
//             ESP_LOGI("TOUCH", "%s: rien (val = %"PRIu32")", boutons_touches[ucNumBouton].nom, val_filtree);
//         }
// }

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

    while (1) 
    {
        if (xQueueReceive(queue_touch_evt, &evt, portMAX_DELAY))
         {
            // int gpio = boutons_touches[evt.pad_num].gpio;
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
                        touch_pad_timeout_resume();// essentiel sinon le FSM arrete de mesurer
                    }
                }
            }
            // switch (gpio)
            // {
            //     case TOUCH_BOUTON_POWER:
            //         if(fryer.state == "OFF")
            //         {
            //             fryer.state = "ON";
            //         }
            //         else
            //         {
            //             fryer.state = "OFF";
            //         }
            //         airfryer_set_etat(&fryer);
            //         break;
            //
            //     case TOUCH_BOUTON_AIRFRY:
            //         fryer.time = "20:00";
            //         fryer.temp = "400";
            //         break;
            //
            //     case TOUCH_BOUTON_DEHYDRATE:
            //         fryer.time = "8h:00";
            //         fryer.temp = "135";
            //         break;
            //
            //     case TOUCH_BOUTON_FRIES:
            //         fryer.time = "15:00";
            //         fryer.temp = "400";
            //         break;
            //
            //     case TOUCH_BOUTON_KEEPWARM:
            //         fryer.time = "30:00";
            //         fryer.temp = "200";
            //         break;
            //    
            //     case TOUCH_BOUTON_NUGGETS:
            //         fryer.time = "15:00";
            //         fryer.temp = "375";
            //         break;
            //
            //     case TOUCH_BOUTON_MINUS:
            //         bouton_Minus_callback();
            //         break;
            //     
            //     case TOUCH_BOUTON_PLUS:
            //         bouton_Plus_callback();
            //         break;
            //
            //     case TOUCH_BOUTON_REHEAT:
            //         fryer.time = "15:00";
            //         fryer.temp = "300";
            //         break;
            //
            //     case TOUCH_BOUTON_START:
            //         bouton_Start_callback();
            //         break;
            //
            //     case TOUCH_BOUTON_STOPCANCEL:
            //         bouton_StopCancel_callback();
            //         break;
            //
            //     case TOUCH_BOUTON_TEMPTIME:
            //         bouton_TempTime_callback();
            //         break;
            //
            //     case TOUCH_BOUTON_PREHEAT:
            //         bouton_Preheat_callback();
            //         break;
            //    
            //     case TOUCH_BOUTON_TURNREMINDER:
            //         bouton_TurnReminder_callback();
            //         break;
            // }
            // for (int i = 0; i < NUM_TOUCH_BUTTONS; ++i) 
            // {
            //     if (boutons_touches[i].pad == evt.pad_num) 
            //     {
            //         if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE)
            //         {
            //             ESP_LOGI(TAG, "TOUCH DÉTECTÉ: %s (pad %"PRIu32")", boutons_touches[i].nom, evt.pad_num);
            //         } 
            //         else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) 
            //         {
            //             ESP_LOGI(TAG, "RELÂCHE: %s (pad %"PRIu32")", boutons_touches[i].nom, evt.pad_num);
            //         } 
            //         else if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) 
            //         {
            //             ESP_LOGW(TAG, "TIMEOUT sur pad %"PRIu32, evt.pad_num);
            //         }
            //     }
            // }
        }
    }
}

// void airfryer_set_etat(const air_fryer_status_t* fryer)
//     {
//       const uint8_t addr = 0x74;
//
//       const uint8_t led_autres[] = {
//         0, 1, 2, 3, 4, 5,
//         18, 19, 20, 21, 22, 23,
//         36, 37, 38, 39,
//         41 // (sauf D3-17 à index 40)
//       };
//
//       is31fl3731_select_page(addr, 0x01);
//
//       // Éteindre toutes les LEDs
//       for (int i = 0; i < 144; i++) 
//       {
//         is31fl3731_write_register(addr, i, 0);
//       }
//
//       // Allumer toujours la LED D3-17 (POWER)
//       is31fl3731_write_register(addr, 40, 255);
//
//       // Si l'état est "ON", allume les autres LEDs
//       if (strcmp(fryer->state, "ON") == 0) 
//       {
//         for (int i = 0; i < sizeof(led_autres); i++) 
//         {
//             is31fl3731_write_register(addr, led_autres[i], 255);
//         }
//         ESP_LOGI("AirFryer", "État ON, LEDs allumées.");
//       } 
//       else
//       {
//         ESP_LOGI("AirFryer", "État %s, seules LED POWER allumée.", fryer->state);
//       }
//    }//fin fonction airfryer_set_etat

void bouton_minus_callback(void)
{

}

void bouton_plus_callback(void)
{
    
}

void bouton_Start_callback(void)
{

}

void bouton_StopCancel_callback(void)
{

}

void bouton_TempTime_callback(void)
{

}

void bouton_Preheat_callback(void)
{

}

void bouton_TurnReminder_callback(void)
{

}

void test_all_leds(uint8_t addr) 
{
    // const uint8_t led_indices[] = {
    //     0, 1, 2, 3, 4, 5,       // CA1 à CA6 sur CB1
    //     18, 19, 20, 21, 22, 23, // CA1 à CA6 sur CB2
    //     36, 37, 38, 39, 40, 41  // CA1 à CA6 sur CB3
    // };
    // const uint8_t led_indices[] = {
    //     0, 1, 2, 3, 4, 5, 6, 7, 8,
    //     9, 10, 11, 12, 13, 14, 15, 16,
    //     17, 18, 19, 20, 21, 22, 23, 24,
    //     25, 26, 27, 28, 29, 30, 31, 32,
    //     33, 34, 35, 36, 37, 38, 39, 40,
    //     41, 42, 43, 44, 45, 46, 47, 48,
    //     49, 50, 51, 52, 53, 54, 55, 56,
    //     57, 58, 59, 60, 61, 62, 63, 64,
    //     65, 66, 67, 68, 69, 70, 71, 72,
    //     73, 74, 75, 76, 77, 78, 79, 
    
    // for (int i = 0; i < sizeof(led_indices); i++)
    // {
        // is31fl3731_light_led(addr, led_indices[i], 255);
    // }
    // for (int i = 0; i < 100; i++)
    // {
    //     is31fl3731_light_led(addr, i, 255);
    //     //ESP_LOGI("valeur de i: %")
    //     printf("valeur de i: %d ; ",i);
    //     vTaskDelay(1000);
    // }
    //is31fl3731_light_led(addr, 38, 255);
    ESP_LOGI("IS31FL3731", "Toutes les LEDs allumées.");
}

esp_err_t is31fl3731_init(uint8_t addr)
 {
    esp_err_t ret;
    // 1. Sélectionner la Function Register page (0x0B)
    ret = is31fl3731_select_page(addr, 0x0B);
    if (ret != ESP_OK) return ret;

    // 2. Sortir du shutdown
    ret = is31fl3731_write_register(I2C_NUM_0, addr, 0x0A, 0x01);
    if (ret != ESP_OK) return ret;

    // 3. Mettre en mode Picture
    ret = is31fl3731_write_register(I2C_NUM_0, addr, 0x00, 0x00);
    if (ret != ESP_OK) return ret;

    // 4. Afficher la frame 1
    ret = is31fl3731_write_register(I2C_NUM_0, addr, 0x01, 0x00);
    if (ret != ESP_OK) return ret;

    printf("initialisation des leds \r");
    // for (int i = 0; i < 100; i++)
    // {
    //     is31fl3731_light_led(addr, i, 0);
    // }
    

    return ESP_OK;
}

esp_err_t is31fl3731_write_register(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;

    esp_err_t ret;
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd, reg, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd, data, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));

cleanup:
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t is31fl3731_select_page(uint8_t addr, uint8_t page)
{
    return is31fl3731_write_register(I2C_NUM_0, addr, 0xFD, page);
}

// esp_err_t is31fl3731_light_led(uint8_t addr, uint8_t led_index, uint8_t brightness) 
// {
//     if (led_index >= 144) return ESP_ERR_INVALID_ARG;
//     is31fl3731_select_page(addr, 0x01);
//     return is31fl3731_write_register(I2C_NUM_0, addr, led_index, brightness);   
// }
esp_err_t is31fl3731_light_led( uint8_t addr, uint8_t indiceCA, uint8_t indiceCB, uint8_t etatCA, uint8_t etatCB)
{
    if (led_index >= 144) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = is31fl3731_select_page(addr, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE("LED", "Erreur select_page");
        return ret;
    }
    ret = is31fl3731_write_register(I2C_NUM_0, addr, indiceCA, etatCA);
    ret = is31fl3731_select_page(addr, 0x00);
    ret = is31fl3731_write_register(I2C_NUM_0, addr, indiceCB, etatCB);

    ESP_LOGI("LED", "LED[%d] ← %d [%s]", led_index, brightness, ret == ESP_OK ? "OK" : "FAIL");

    return ret;
}
