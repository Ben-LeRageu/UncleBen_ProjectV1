#include "is31fl3731.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "IS31FL3731"
#define I2C_TIMEOUT_MS 100



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

esp_err_t is31fl3731_init(uint8_t addr)
{
    uint8_t addresse_ledCA[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10};
    uint8_t addresse_ledCB[9] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x11};
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

    ret = is31fl3731_select_page(addr, 0x00);
    for(int i=0;i<8;i++)
    {
        ret = is31fl3731_write_register(I2C_NUM_0, addr, addresse_ledCA[i], 0x00);
        if (ret != ESP_OK) return ret;
    }
    for(int i=0;i<3;i++)
    {
        ret = is31fl3731_write_register(I2C_NUM_0, addr, addresse_ledCB[i], 0xFF);
        if (ret != ESP_OK) return ret;
    }
    for(int i=3;i<8;i++)
    {
        ret = is31fl3731_write_register(I2C_NUM_0, addr, addresse_ledCB[i], 0x00);//mets les pins de CB non connectées à 0
        if (ret != ESP_OK) return ret;
    }
//    for(int i=0x24; i<0xAC; i++)
  //  {
    //    ret = is31fl3731_write_register(I2C_NUM_0, addr, i, 0xDF);//pwm
    //    if (ret != ESP_OK) return ret;
    //}
    
    printf("initialisation des leds \r");
    return ESP_OK;
}

esp_err_t is31fl3731_light_ledCA(uint8_t addr, uint8_t indiceCA, uint8_t etatCA)
{
    //esp_err_t ret = is31fl3731_select_page(addr, 0x00);
            // if (ret != ESP_OK) {
            //     ESP_LOGE("LED", "Erreur select_page");
            //     return ret;
    //}
    esp_err_t ret = is31fl3731_write_register(I2C_NUM_0, addr, indiceCA, etatCA);

    // ESP_LOGI("LED", "LED CA[%d]=%d [%s]", 
    //          indiceCA, etatCA,
    //          ret == ESP_OK ? "OK" : "FAIL");

    return ret;
}

esp_err_t is31fl3731_light_ledCB(uint8_t addr, uint8_t indiceCB, uint8_t etatCB)
{
    //esp_err_t ret = is31fl3731_select_page(addr, 0x00);
    // if (ret != ESP_OK) {
    //     ESP_LOGE("LED", "Erreur select_page");
    //     return ret;
    // }
    esp_err_t ret = is31fl3731_write_register(I2C_NUM_0, addr, indiceCB, etatCB);
    // ESP_LOGI("LED", "LED CB[%d]=%d [%s]", 
    //          indiceCB, etatCB,
    //          ret == ESP_OK ? "OK" : "FAIL");
    return ret;
}

//void test_all_leds(uint8_t addr) 
//{
//    ESP_LOGI("IS31FL3731", "Toutes les LEDs allumées.");
//}
