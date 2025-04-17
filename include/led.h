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