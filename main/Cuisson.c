#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Cuisson.h"
#include "air_fryer.h"

// Définition des broches selon le schéma de l'ESP32-S3
#define HEATER_GPIO     GPIO_NUM_35    // Relais pour l'élément chauffant
#define FAN_CTRL_GPIO   GPIO_NUM_36    // OPTO-TRIAC pour le ventilateur
#define NTC_ADC_CHANNEL ADC2_CHANNEL_4 // Pour la lecture NTC

// Configuration LEDC pour le relais
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT // Résolution de 13 bits (0-8191)
#define LEDC_FREQUENCY         10                // 10 Hz
#define LEDC_DUTY_MAX         8191               // Valeur maximale pour 13 bits

// Configuration du ventilateur
#define FAN_PULSE_US     100           // Durée de l'impulsion pour l'OPTO-TRIAC
#define FAN_PERIOD_MS    10            // Période entre les impulsions (100Hz)

static const char *TAG = "CUISSON";
static bool cuisson_active = false;
static TaskHandle_t tache_maintien = NULL;
static TaskHandle_t tache_fan = NULL;
static float puissance_actuelle = 0.0;  // 0.0 à 1.0
static bool fan_active = false;

// Tâche pour générer les impulsions du ventilateur
static void fan_pulse_task(void *arg) {
    while (1) {
        if (fan_active) {
            // Génère une impulsion courte
            gpio_set_level(FAN_CTRL_GPIO, 1);
            esp_rom_delay_us(FAN_PULSE_US);
            gpio_set_level(FAN_CTRL_GPIO, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(FAN_PERIOD_MS));
    }
}

static float lire_temperature()
{
    int raw; 
    adc2_get_raw(NTC_ADC_CHANNEL, 12, &raw );
    float tension = (float)raw / 4095.0 * 3.3;
    float temperature = (100.0 - (tension * 100)); // Approximation simple
    return temperature;
}

static void tache_maintien_temp(void *param) {
    float kp = 0.1f;  // Coefficient proportionnel
    float ki = 0.01f; // Coefficient intégral
    float erreur_integrale = 0.0f;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (cuisson_active) {
        float t = lire_temperature();
        float erreur = fryer.temp - t;
        erreur_integrale += erreur;
        
        // Calcul de la puissance avec PI
        puissance_actuelle = kp * erreur + ki * erreur_integrale;
        
        // Limites de la puissance entre 0 et 1
        if (puissance_actuelle > 1.0f) puissance_actuelle = 1.0f;
        if (puissance_actuelle < 0.0f) puissance_actuelle = 0.0f;

        // Mise à jour du duty cycle LEDC
        uint32_t duty = (uint32_t)(puissance_actuelle * LEDC_DUTY_MAX);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        ESP_LOGI(TAG, "Temp: %.2f°F / Cible: %d°F / Puissance: %.2f%%", 
                 t, fryer.temp, puissance_actuelle * 100.0f);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void cuisson_gpio_init()
{
    // Configuration des GPIOs pour le ventilateur
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FAN_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configuration du timer LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configuration du canal LEDC
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = HEATER_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    // Configuration de l'ADC pour la lecture NTC
    //adc2_config_width(ADC_WIDTH_BIT_12);
    adc2_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // État initial : tout éteint
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    gpio_set_level(FAN_CTRL_GPIO, 0);

    // Création de la tâche du ventilateur
    xTaskCreatePinnedToCore(fan_pulse_task, "FAN_Task", 2048, NULL, 5, &tache_fan, 0);

    ESP_LOGI(TAG, "Initialisation du contrôle de cuisson terminée");
}

void start_cuisson() 
{
    fan_active = true;     // Active le ventilateur
    cuisson_active = true; // Active le chauffage
    ESP_LOGI(TAG, "Cuisson demarree");
}

void stop_cuisson() 
{
    cuisson_active = false;
    fan_active = false;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    gpio_set_level(FAN_CTRL_GPIO, 0);
    if (tache_maintien) {
        vTaskDelete(tache_maintien);
        tache_maintien = NULL;
    }
    ESP_LOGI(TAG, "Cuisson arretee");
}

void maintenir_temperature(float temperature_cible) {
    if (!cuisson_active) {
        start_cuisson();
    }

    if (tache_maintien == NULL) {
        xTaskCreatePinnedToCore(tache_maintien_temp, "MaintienTemp", 1024, NULL, 5, &tache_maintien, 1);
    }
}
