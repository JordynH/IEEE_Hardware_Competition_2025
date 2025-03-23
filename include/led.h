/**
 * @file led.h
 * @brief LED setup and control functions
 * 
 * This file declares the functions to intialize and control LEDs using the LEDC peripheral on the ESP32..
 * 
 * @author Solomon Tolson
 * @version 1.0
 * @date 2025-03-03
 * @modified 2025-03-17
 */

#ifndef LED_H
#define LED_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Struct to hold LED configuration
 * 
 * Components:
 * - channel: LEDC channel configuration
 * - timer: LEDC timer configuration
 * - pin: GPIO pin number
 * - group: LEDC group
 */
typedef struct {
    gpio_num_t pin;
} led_t;

/**
 * @brief Initialize an LED configuration
 * 
 * @param led Pointer to the led_t struct.
 */
void led_init(led_t *led);

/**
 * @brief Set the brightness of the LED
 * 
 * @param led Pointer to the led_t struct.
 * @param brightness Brightness value (0-100).
 */
void led_set_brightness(led_t *led, int brightness);

void led_flash(led_t *led);

#endif // LED_H