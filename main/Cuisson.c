#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Cuisson.h"
#include "air_fryer.h"

// Définition des broches selon le schéma de l'ESP32-S3
#define HEATER_GPIO     GPIO_NUM_35    // Relais pour l'élément chauffant
#define FAN_CTRL_GPIO   GPIO_NUM_36    // OPTO-TRIAC pour le ventilateur
#define NTC_ADC_CHANNEL ADC2_CHANNEL_4 // Pour la lecture NTC

// Pour le relais, on utilise une fréquence plus basse pour éviter l'usure
#define RELAY_FREQ_HZ    1             // 1 Hz = cycle d'1 seconde
#define RELAY_PERIOD_MS  (1000 / RELAY_FREQ_HZ)
#define FAN_PULSE_US     100           // Durée de l'impulsion pour l'OPTO-TRIAC

static const char *TAG = "CUISSON";
static bool cuisson_active = false;
static TaskHandle_t tache_maintien = NULL;
static TaskHandle_t tache_pwm = NULL;
static TaskHandle_t tache_fan = NULL;
static float puissance_actuelle = 0.0;  // 0.0 à 1.0
static bool fan_active = false;

// Tâche pour générer les impulsions du ventilateur
static void fan_pulse_task(void *arg) {
    while (1) {
        if (fan_active) {
            gpio_set_level(FAN_CTRL_GPIO, 1);
            esp_rom_delay_us(FAN_PULSE_US);
            gpio_set_level(FAN_CTRL_GPIO, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz pour les impulsions
    }
}

// Tâche pour le contrôle PWM du relais de chauffage
static void heater_control_task(void *arg) {
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(RELAY_PERIOD_MS);
    
    while (1) {
        xLastWakeTime = xTaskGetTickCount();
        
        if (cuisson_active && puissance_actuelle > 0.0f) {
            // ON pendant 800ms, OFF pendant 200ms (80% duty cycle)
            gpio_set_level(HEATER_GPIO, 1);
            ESP_LOGI(TAG, "Relais ON");
            vTaskDelay(pdMS_TO_TICKS(RELAY_PERIOD_MS * 0.8));
            
            gpio_set_level(HEATER_GPIO, 0);
            ESP_LOGI(TAG, "Relais OFF");
            vTaskDelay(pdMS_TO_TICKS(RELAY_PERIOD_MS * 0.2));
        } else {
            gpio_set_level(HEATER_GPIO, 0);
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
}

static float lire_temperature()
{
    int raw = adc1_get_raw(NTC_ADC_CHANNEL);
    float tension = (float)raw / 4095.0 * 3.3;
    float temperature = (100.0 - (tension * 100)); // Approximation simple
    return temperature;
}

static void tache_maintien_temp(void *param) {
    // On n'a plus besoin du paramètre car on utilise fryer.temp
    //UNUSED_PARAMETER(param);

    // PID simple
    float kp = 0.1f;  // Coefficient proportionnel
    float ki = 0.01f; // Coefficient intégral
    float erreur_integrale = 0.0f;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (cuisson_active) {
        float t = lire_temperature();
        float erreur = fryer.temp - t;  // Utilisation de fryer.temp comme température cible
        erreur_integrale += erreur;
        
        // Calcul de la puissance avec PI
        puissance_actuelle = kp * erreur + ki * erreur_integrale;
        
        // Limites de la puissance entre 0 et 1
        if (puissance_actuelle > 1.0f) puissance_actuelle = 1.0f;
        if (puissance_actuelle < 0.0f) puissance_actuelle = 0.0f;

        ESP_LOGI(TAG, "Temp: %.2f°C / Cible: %d°C / Puissance: %.2f%%", 
                 t, fryer.temp, puissance_actuelle * 100.0f);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void cuisson_gpio_init()
{
    // Configuration des GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HEATER_GPIO) | (1ULL << FAN_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configuration de l'ADC pour la lecture NTC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // État initial : tout éteint
    gpio_set_level(HEATER_GPIO, 0);
    gpio_set_level(FAN_CTRL_GPIO, 0);

    // Création des tâches
    xTaskCreatePinnedToCore(heater_control_task, "Heater_Task", 2048, NULL, 5, &tache_pwm, 0);
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
    gpio_set_level(HEATER_GPIO, 0);
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
        // Création de la tâche sans paramètre de température
        xTaskCreatePinnedToCore(tache_maintien_temp, "MaintienTemp", 1024, NULL, 5, &tache_maintien, 1);
    }
}
