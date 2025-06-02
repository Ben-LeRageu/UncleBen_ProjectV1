/*
 * Copyright (c) 2022 Timofei Korostelev <timofei_public@dranik.dev>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file ht16k33.h
 * @defgroup ht16k33 ht16k33
 * @{
 *
 * Holtek HT16K33 LED Controller driver.
 * No keyscan features implemented.
 * Tip: PWM brightness frequency depends on I2C clock.
 *
 * Manufacturer link: https://www.holtek.com/productdetail/-/vg/HT16K33
 * Datasheet: https://www.holtek.com/documents/10179/116711/HT16K33v120.pdf
 *
 */

#ifndef __HT16K33_H__
#define __HT16K33_H__

#include <esp_err.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le HT16K33
 * @param addr Adresse I2C du périphérique
 * @return ESP_OK en cas de succès
 */
esp_err_t ht16k33_init(uint8_t addr);

/**
 * @brief Configure la luminosité de l'afficheur
 * @param addr Adresse I2C du périphérique
 * @param brightness Luminosité (0-15)
 * @return ESP_OK en cas de succès
 */
esp_err_t ht16k33_set_brightness(uint8_t addr, uint8_t brightness);

/**
 * @brief Configure l'affichage (on/off et clignotement)
 * @param addr Adresse I2C du périphérique
 * @param on 1 pour allumer, 0 pour éteindre
 * @param blink Mode de clignotement (0-3)
 * @return ESP_OK en cas de succès
 */
esp_err_t ht16k33_display_setup(uint8_t addr, uint8_t on, uint8_t blink);

/**
 * @brief Écrit des données dans la RAM du HT16K33
 * @param addr Adresse I2C du périphérique
 * @param reg Adresse de début dans la RAM
 * @param data Pointeur vers les données
 * @param size Taille des données
 * @return ESP_OK en cas de succès
 */
esp_err_t ht16k33_write_data(uint8_t addr, uint8_t reg, uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HT16K33_H__
