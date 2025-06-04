#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Cuisson.h"
#include "air_fryer.h"

// Définition des broches selon le schéma de l'ESP32-S3
#define HEATER_GPIO     GPIO_NUM_35    // Relais pour l'élément chauffant
#define FAN_CTRL_GPIO   GPIO_NUM_36    // OPTO-TRIAC pour le ventilateur
#define NTC_ADC_CHANNEL ADC2_CHANNEL_4 // Pour la lecture NTC

// Configuration MCPWM
#define MCPWM_TIMER          MCPWM_TIMER_0
#define MCPWM_GEN           MCPWM_OPR_A
#define MCPWM_UNIT          MCPWM_UNIT_0
#define MCPWM_FREQUENCY     2000        // 2 kHz
#define MCPWM_DUTY_MAX     100.0       // Pourcentage maximum

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
    float kp = 0.05f;  // Réduit pour une réponse plus douce
    float ki = 0.001f; // Réduit pour éviter l'accumulation rapide
    float kd = 0.01f;  // Ajout d'un terme dérivé pour la stabilité
    float erreur_integrale = 0.0f;
    float erreur_precedente = 0.0f;
    const float MAX_INTEGRAL = 50.0f; // Limite l'accumulation de l'intégrale
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (cuisson_active) {
        float t = lire_temperature();
        float erreur = fryer.temp - t;
        
        // Anti-windup : limite l'accumulation de l'erreur intégrale
        erreur_integrale += erreur;
        if (erreur_integrale > MAX_INTEGRAL) erreur_integrale = MAX_INTEGRAL;
        if (erreur_integrale < -MAX_INTEGRAL) erreur_integrale = -MAX_INTEGRAL;
        
        // Terme dérivé
        float erreur_derivee = (erreur - erreur_precedente);
        erreur_precedente = erreur;
        
        // Calcul de la puissance avec PID
        puissance_actuelle = kp * erreur + 
                           ki * erreur_integrale + 
                           kd * erreur_derivee;
        
        // Limites de la puissance entre 0 et 1
        if (puissance_actuelle > 1.0f) puissance_actuelle = 1.0f;
        if (puissance_actuelle < 0.0f) puissance_actuelle = 0.0f;

        // Limite supplémentaire de sécurité : maximum 80% de puissance
        if (puissance_actuelle > 0.5f) puissance_actuelle = 0.5f;

        // Mise à jour du duty cycle MCPWM
        float duty = puissance_actuelle * MCPWM_DUTY_MAX;
        ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, duty));
        
        ESP_LOGI(TAG, "Temp: %.2f°F / Cible: %d°F / Erreur: %.2f / I: %.2f / D: %.2f / Puissance: %.2f%%", 
                 t, fryer.temp, erreur, erreur_integrale, erreur_derivee, puissance_actuelle * 100.0f);

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

    // Configuration MCPWM
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT, MCPWM_GEN, HEATER_GPIO));

    mcpwm_config_t pwm_config = {
        .frequency = MCPWM_FREQUENCY,
        .cmpr_a = 0,                    // Duty cycle initial de 0%
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_DOWN_COUNTER,  // Mode compteur up/down pour un signal plus stable
    };
    
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config));
    
    // Configuration des paramètres du générateur PWM
    ESP_ERROR_CHECK(mcpwm_set_duty_type(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, MCPWM_DUTY_MODE_0));
    
    // Démarrer avec un duty cycle de 0%
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT, MCPWM_TIMER));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, 0));

    // Forcer une mise à jour immédiate du duty cycle
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, 0));

    // Configuration de l'ADC pour la lecture NTC
    adc2_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // État initial : tout éteint
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
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, 0));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, 0));
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
