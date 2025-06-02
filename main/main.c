#include <stdio.h>
#include <inttypes.h>
#include <string.h>  // Pour strcmp et strcpy
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/touch_pad.h"
#include "freertos/queue.h"
#include "is31fl3731.h"
#include "Cuisson.h"
#include "air_fryer.h"  // Ajout du nouveau fichier d'en-tête
#include "ht16k33.h"

// Définition des adresses I2C pour les afficheurs HT16K33
#define HT16K33_ADDR_TIME     0x70  // Adresse pour l'afficheur de temps
#define HT16K33_ADDR_TEMP     0x75  // Adresse pour l'afficheur de température
#define DIGIT1 0x02
#define DIGIT2 0x04
#define DIGIT3 0x06
#define DIGIT4 0x08
#define DIGIT_L1_L2 0x0A
#define DIGIT_L3 0x0C

// Déclaration globale de la variable fryer
air_fryer_status_t fryer = {
    .temp = 0,
    .total_minutes = 0,
    .time_display = "00:00",
    .state = "OFF",
    .mode = "AP"
};

#define I2C_TIMEOUT_MS 100
#define I2C_MASTER_SCL_IO  GPIO_NUM_21  // adapte selon ton schéma
#define I2C_MASTER_SDA_IO  GPIO_NUM_20  // adapte selon ton schéma
#define I2C_MASTER_NUM     I2C_NUM_0    // I2C port
#define I2C_MASTER_FREQ_HZ 400000       // 400kHz comme avant
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// Prototypes
esp_err_t i2c_master_init(void);
void test_all_leds(uint8_t addr);
void init_touch_buttons(void);
void lire_touches_tactiles(void);
void lire_une_touches_tactile(unsigned char ucNumBouton);
void calibrer_touch_seuils(void);
void touch_task(void *param);
static void touch_isr_cb(void *arg);
void airfryer_set_etat(air_fryer_status_t* fryer);
void bouton_Minus_callback(void);
void bouton_Plus_callback(void);
void bouton_Start_callback(void);
void bouton_StopCancel_callback(void);
void bouton_TempTime_callback(bool *temptime);
void bouton_Preheat_callback(void);
void bouton_TurnReminder_callback(void);
void update_time_display(air_fryer_status_t* fryer);
esp_err_t init_ht16k33_displays(void);  // Nouveau prototype
void display_temperature(uint16_t temp);

#define TOUCH_THRESH_PERCENT  (2)  // Seuil à 2% de la valeur au repos
#define NUM_TOUCH_BUTTONS     14
#define TEMP 1
#define TIME 0

static const char *TAG = "TOUCH_MAIN";
static QueueHandle_t queue_touch_evt = NULL;
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS];
static bool temptime = TEMP;

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

// Valeurs au repos
static uint32_t valeurs_repos[NUM_TOUCH_BUTTONS] = {0};
uint8_t addresse_ledCA[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10};
uint8_t addresse_ledCB[9] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x11};

typedef struct {
    uint8_t reg;
    uint8_t regp1;
} digit_pattern_t;

static const digit_pattern_t digit_segments[11] = {
    {0b11111000, 0b00000001}, // 0 → A B C D E F
    {0b01100000, 0b00000000}, // 1 → B C
    {0b11011000, 0b00000010}, // 2 → A B D E G
    {0b11110000, 0b00000010}, // 3 → A B C D G
    {0b01100100, 0b00000010}, // 4 → B C F G
    {0b10110100, 0b00000010}, // 5 → A C D F G
    {0b10111100, 0b00000010}, // 6 → A C D E F G
    {0b11100000, 0b00000000}, // 7 → A B C
    {0b11111100, 0b00000011}, // 8 → A B C D E F G
    {0b11110100, 0b00000010}, // 9 → A B C D F G
    {0b10001000, 0b00000011}  // F → A E F G
};
#define LETTRE_F digit_segments[10]

void app_main(void)
{
    esp_err_t ret;
    
    // Initialisation GPIO pour la cuisson
    cuisson_gpio_init();
    
    // Initialisation I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Échec de l'initialisation I2C: %d", ret);
        return;
    }
    
    // Petit délai après l'initialisation I2C
    esp_rom_delay_us(100000);
    
    // Initialisation des afficheurs HT16K33 d'abord
    ret = init_ht16k33_displays();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Échec de l'initialisation des afficheurs HT16K33");
        return;
    }
    
    // Affichage initial de la température
    display_temperature(fryer.temp);
    
    // Petit délai entre les initialisations
    esp_rom_delay_us(100000);
    
    // Initialisation de l'IS31FL3731
    uint8_t addr = 0x74;
    if (is31fl3731_init(addr) != ESP_OK) {
        ESP_LOGE("MAIN", "Échec de l'initialisation IS31FL3731");
        return;
    }
    
    // Configuration initiale de l'état
    airfryer_set_etat(&fryer);
    
    // Initialisation de la file d'attente pour les événements tactiles
    if(queue_touch_evt == NULL) {
        queue_touch_evt = xQueueCreate(16, sizeof(touch_evt_t));
        if(queue_touch_evt == NULL) {
            ESP_LOGE("MAIN", "Échec de la création de la file d'attente des événements tactiles");
            return;
        }
    }
    
    // Initialisation des boutons tactiles
    init_touch_buttons();
    
    // Création de la tâche de gestion des touches
    xTaskCreate(touch_task, "touch_evt_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI("MAIN", "Initialisation complète réussie");
}

//fonction init I2C
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Erreur de configuration I2C: %d", err);
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Erreur d'installation du driver I2C: %d", err);
        return err;
    }

    return ESP_OK;
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
            touch_pad_t touch_num = evt.pad_num;  // On utilise directement le numéro du pad
            if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE)
            {
            switch (touch_num)
            {
                // Suivre exactement l'ordre de l'énumération touch_bouton_t
                case TOUCH_PAD_NUM10:  // AIRFRY
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 20;
                    fryer.temp = 400;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM2:   // FRIES
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 15;
                    fryer.temp = 400;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM3:   // REHEAT
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 15;
                    fryer.temp = 300;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM9:   // MINUS
                if(strcmp(fryer.state, "ON") == 0)
                {
                    bouton_Minus_callback();
                    break;
                }
                else break;
                

                case TOUCH_PAD_NUM7:   // TEMPTIME
                if(strcmp(fryer.state, "ON") == 0)
                {
                    bouton_TempTime_callback(&temptime);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM8:   // PLUS
                if(strcmp(fryer.state, "ON") == 0)
                {
                    bouton_Plus_callback();
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM6:   // NUGGETS
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 15;
                    fryer.temp = 375;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM1:   // DEHYDRATE
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 480;  // 8 heures
                    fryer.temp = 135;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;
                
                case TOUCH_PAD_NUM5:   // KEEPWARM
                if(strcmp(fryer.state, "ON") == 0)
                {
                    fryer.total_minutes = 30;
                    fryer.temp = 200;
                    update_time_display(&fryer);
                    display_temperature(fryer.temp);
                    break;
                }
                else break;

                case TOUCH_PAD_NUM14:  // PREHEAT
                if(strcmp(fryer.state, "ON") == 0)
                {
                    bouton_Preheat_callback();
                    break;
                }
                else break;

                case TOUCH_PAD_NUM13:  // STOPCANCEL
                if((strcmp(fryer.state, "ON") == 0) || (strcmp(fryer.state, "cooking") == 0))
                {
                    bouton_StopCancel_callback();
                    break;
                }
                else break;

                case TOUCH_PAD_NUM4:   // POWER
                if(strcmp(fryer.state, "OFF") == 0)
                    {
                        strcpy(fryer.state, "ON");
                    }
                    else
                    {
                        strcpy(fryer.state, "OFF");
                    }
                    airfryer_set_etat(&fryer);
                    break;

                case TOUCH_PAD_NUM12:  // START
                if((strcmp(fryer.state, "ON") == 0) || (strcmp(fryer.state, "stopped") == 0))
                {
                    bouton_Start_callback();
                    break;
                }
                else break; 

                case TOUCH_PAD_NUM11:  // TURNREMINDER
                if(strcmp(fryer.state, "ON") == 0)
                {
                    bouton_TurnReminder_callback();
                    break;
                }
                else break;

                default:
                    break;
            }
            }
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

void airfryer_set_etat(air_fryer_status_t* fryer)
{
    uint8_t addr = 0x74;
    // Sélectionner la page une seule fois au début
    is31fl3731_select_page(addr, 0x00);

    if(strcmp(fryer->state, "ON") == 0)
    {
        // Configurer toutes les LEDs CA
        for(int i = 0; i < 6; i++)
        {
            is31fl3731_light_ledCA(addr, addresse_ledCA[i], 0xFF);
        }
        // Configurer toutes les LEDs CB
        for(int i = 0; i < 3; i++)
        {
            //is31fl3731_light_ledCB(addr, addresse_ledCB[i], 0x00);
        }
        ESP_LOGI("AirFryer", "État ON, LEDs allumées.");
    }
    else if(strcmp(fryer->state, "OFF") == 0)
    {
        // Configurer toutes les LEDs CA
        for(int i = 0; i < 6; i++)
        {
            is31fl3731_light_ledCA(addr, addresse_ledCA[i], 0x00);
        }
        // Configurer toutes les LEDs CB
        for(int i = 0; i < 3; i++)
        {
            //is31fl3731_light_ledCB(addr, addresse_ledCB[i], 0x01);
        }
        ESP_LOGI("AirFryer", "État %s, seule LED POWER allumée.", fryer->state);
    }
    // Allumer toujours la LED D3-17 (POWER) - pas besoin de resélectionner la page
    //is31fl3731_light_ledCA(addr, 0x10, 0x01);
    is31fl3731_light_ledCA(addr, 0x04, 0x08);//lum bouton power
    //is31fl3731_light_ledCB(addr, 0x05, 0x00);
}

void bouton_Minus_callback(void)
{
    if(temptime == TEMP) 
    {
        if(fryer.temp > 100)
        {
            fryer.temp -= 25;
            display_temperature(fryer.temp);
            ESP_LOGI("TEMP", "Temperature diminuée à %d°F", fryer.temp);
        }
    }
    else
    {
        fryer.total_minutes--;
        update_time_display(&fryer);
        ESP_LOGI("TIME", "Temps restant: %s", fryer.time_display);
    }
}

void bouton_Plus_callback(void)
{
    if((temptime == TEMP) && (fryer.temp < 400))
    {
        fryer.temp += 25;
        display_temperature(fryer.temp);
        ESP_LOGI("TEMP", "Temperature augmentée à %d°F", fryer.temp);
    }
    else
    {
        fryer.total_minutes++;
        update_time_display(&fryer);
        ESP_LOGI("TIME", "Temps restant: %s", fryer.time_display);
    }
}

void bouton_Start_callback(void)
{
    maintenir_temperature((float)fryer.temp);
    strcpy(fryer.state, "cooking");
    airfryer_set_etat(&fryer);
}

void bouton_StopCancel_callback(void)
{
    stop_cuisson();
    strcpy(fryer.state, "stopped");
    airfryer_set_etat(&fryer);
}

void bouton_TempTime_callback(bool *temptime)
{
    if(*temptime == TEMP)
    {
        *temptime = TIME;
        ESP_LOGI("MODE", "Mode: TEMPS (actuel: %s)", fryer.time_display);
    }
    else
    {
        *temptime = TEMP;
        ESP_LOGI("MODE", "Mode: TEMPERATURE (actuelle: %d°C)", fryer.temp);
    }
}

void bouton_Preheat_callback(void)
{

}

void bouton_TurnReminder_callback(void)
{

}

void test_all_leds(uint8_t addr) 
{
    
    ESP_LOGI("IS31FL3731", "Toutes les LEDs allumées.");
}
// Fonction pour mettre à jour l'affichage du temps
void update_time_display(air_fryer_status_t* fryer) 
{
    if (fryer->total_minutes < 100) 
    {
        // Format MM:SS
        int minutes = fryer->total_minutes;
        int seconds = 0;  // Pour l'instant, on garde les secondes à 0
        snprintf(fryer->time_display, sizeof(fryer->time_display), "%02d:%02d", minutes, seconds);
    } 
    else 
    {
        // Format HhMM
        int hours = fryer->total_minutes / 60;
        int minutes = fryer->total_minutes % 60;
        snprintf(fryer->time_display, sizeof(fryer->time_display), "%1dh%02d", hours, minutes);
    }
}

// Fonction d'initialisation des afficheurs HT16K33
esp_err_t init_ht16k33_displays(void)
{
    esp_err_t ret;
    
    ESP_LOGI("HT16K33", "Début de l'initialisation des afficheurs");
    
    // Initialisation de l'afficheur de temps
    ret = ht16k33_init(HT16K33_ADDR_TIME);
    if (ret != ESP_OK) {
        ESP_LOGE("HT16K33", "Échec de l'initialisation de l'afficheur de temps");
        return ret;
    }
    
    // Initialisation de l'afficheur de température
    ret = ht16k33_init(HT16K33_ADDR_TEMP);
    if (ret != ESP_OK) {
        ESP_LOGE("HT16K33", "Échec de l'initialisation de l'afficheur de température");
        return ret;
    }
    
    ESP_LOGI("HT16K33", "Initialisation des afficheurs réussie");
    return ESP_OK;
}

// Fonction pour afficher la température sur l'afficheur HT16K33
void display_temperature(uint16_t temp) 
{   
    uint8_t ram[16] = {0};
    // Calcul des chiffres individuels
    uint8_t digit1 = temp / 100;       // Centaines
    uint8_t digit2 = (temp / 10) % 10; // Dizaines
    uint8_t digit3 = temp % 10;        // Unités
    digit_pattern_t p1 = digit_segments[digit1];
    digit_pattern_t p2 = digit_segments[digit2];
    digit_pattern_t p3 = digit_segments[digit3];
    digit_pattern_t p4 = LETTRE_F;
    ram[DIGIT1]     = p1.reg;
    ram[DIGIT1 + 1] = p1.regp1;
    ram[DIGIT2]     = p2.reg;
    ram[DIGIT2 + 1] = p2.regp1;
    ram[DIGIT3]     = p3.reg;
    ram[DIGIT3 + 1] = p3.regp1;
    ram[DIGIT4]     = p4.reg;
    ram[DIGIT4 + 1] = p4.regp1;
    
    // Écriture des données sur l'afficheur
    esp_err_t ret = ht16k33_write_data(HT16K33_ADDR_TEMP, 0x00, ram, 16);
    //ret = ht16k33_write_data(HT16K33_ADDR_TEMP, DIGIT2, digit2, 16);
    //ret = ht16k33_write_data(HT16K33_ADDR_TEMP, DIGIT3, digit3, 16);
    //ret = ht16k33_write_data(HT16K33_ADDR_TEMP, DIGIT4, LETTRE_F, 16);
    
    if (ret != ESP_OK) {
        ESP_LOGE("TEMP", "Erreur lors de l'écriture sur l'afficheur: %d", ret);
    }
}
