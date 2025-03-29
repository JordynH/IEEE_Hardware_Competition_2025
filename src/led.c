/**
 * @file led.c
 * 
 * This file defines and implements the functions to initialize and control LEDs using the LEDC peripheral on the ESP32.
 * 
 * @author Solomon Tolson
 * @version 1.0
 * @date 2025-03-03
 * @modified 2025-03-17
 */

#include "led.h"
#include "math.h"

#define TAG "LED"

void led_init(led_t *led) {
    // Configure LED timer
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_NUM_13,
        .duty           = 0,
        .hpoint         = 0
    };
    esp_err_t err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) {
        ESP_LOGE("LED", "led channel config failed: %s", esp_err_to_name(err));
    }
}

void led_set_brightness(led_t *led, int brightness) {
    if (brightness < 0) {
        brightness = 0;
    } else if (brightness > 100) {
        brightness = 100;
    }
    double k = 0.03;
    int input_min = 0;
    int input_max = 100;

    double exp_min = exp(k * (input_min - input_min));
    double exp_max = exp(k * (input_max - input_min));
    double exp_value = exp(k * (brightness - input_min));

    double duty = 451 + (exp_value - exp_min) * (175) / (exp_max - exp_min);
    if (brightness == 0) duty = 400;

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)duty);
    if (err != ESP_OK) {
        ESP_LOGE("LED", "set duty failed: %s", esp_err_to_name(err));
    } else {
        // ESP_LOGI("LED", "duty set to %d", (int)duty);
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (err != ESP_OK) {
        ESP_LOGE("LED", "update duty failed: %s", esp_err_to_name(err));
    }
}

void led_flash(led_t *led) {
    led_set_brightness(led, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_set_brightness(led, 100);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_set_brightness(led, 50);
}