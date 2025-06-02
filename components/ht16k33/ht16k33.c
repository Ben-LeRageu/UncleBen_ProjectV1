#include "ht16k33.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "HT16K33"
#define I2C_TIMEOUT_MS 100

// Commandes HT16K33
#define HT16K33_CMD_SYSTEM_SETUP    0x20
#define HT16K33_CMD_DISPLAY_SETUP   0x80
#define HT16K33_CMD_BRIGHTNESS      0xE0
#define HT16K33_CMD_ROW_INT         0x40

static esp_err_t ht16k33_write_register(uint8_t addr, uint8_t cmd) 
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    if (!cmd_handle) return ESP_FAIL;

    esp_err_t ret;
    ret = i2c_master_start(cmd_handle);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd_handle, (addr << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd_handle, cmd, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_stop(cmd_handle);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));

cleanup:
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

esp_err_t ht16k33_init(uint8_t addr)
{
    esp_err_t ret;

    // 1. Internal system clock enable
    ret = ht16k33_write_register(addr, HT16K33_CMD_SYSTEM_SETUP | 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lors de l'activation de l'horloge système");
        return ret;
    }

    // 2. ROW/INT output pin set & INT pin output level set
    ret = ht16k33_write_register(addr, HT16K33_CMD_ROW_INT | 0x00);  // ROW output
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lors de la configuration des pins ROW/INT");
        return ret;
    }

    // 3. Dimming set
    ret = ht16k33_write_register(addr, HT16K33_CMD_BRIGHTNESS | 0x0F);  // Maximum brightness
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lors du réglage de la luminosité");
        return ret;
    }

    // 4. Blinking set
    ret = ht16k33_write_register(addr, HT16K33_CMD_DISPLAY_SETUP | 0x01);  // Display ON, no blinking
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lors de la configuration du clignotement");
        return ret;
    }

    ESP_LOGI(TAG, "Initialisation réussie pour l'adresse 0x%02X", addr);
    return ESP_OK;
}

esp_err_t ht16k33_set_brightness(uint8_t addr, uint8_t brightness)
{
    if (brightness > 15) {
        ESP_LOGE(TAG, "La luminosité doit être entre 0 et 15");
        return ESP_ERR_INVALID_ARG;
    }
    return ht16k33_write_register(addr, HT16K33_CMD_BRIGHTNESS | brightness);
}

esp_err_t ht16k33_display_setup(uint8_t addr, uint8_t on, uint8_t blink)
{
    if (on > 1 || blink > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    return ht16k33_write_register(addr, HT16K33_CMD_DISPLAY_SETUP | (blink << 1) | on);
}

esp_err_t ht16k33_write_data(uint8_t addr, uint8_t reg, uint8_t *data, size_t size)
{
    esp_err_t ret;

    // 1. Address setting
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;

    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) goto cleanup;

    // Commande pour définir l'adresse de départ dans la RAM (0x00-0x0F)
    ret = i2c_master_write_byte(cmd, reg & 0x0F, true);
    if (ret != ESP_OK) goto cleanup;

    // 2. Display data RAM write
    ret = i2c_master_write(cmd, data, size, true);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) goto cleanup;

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) goto cleanup;

    // 3. Display on (s'assurer que l'afficheur est allumé après l'écriture)
    ret = ht16k33_write_register(addr, HT16K33_CMD_DISPLAY_SETUP | 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lors de l'activation de l'afficheur après écriture");
        goto cleanup;
    }

cleanup:
    i2c_cmd_link_delete(cmd);
    return ret;
} 