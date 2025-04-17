// Uncomment the code below and comment out the code in main.cpp to test the encoder functionality

#include <stdio.h>
#include "driver/pcnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"

#define TAG "ENCODER_TEST"

// Define PCNT units for each motor
#define PCNT_UNIT_FR PCNT_UNIT_0
#define PCNT_UNIT_FL PCNT_UNIT_1
#define PCNT_UNIT_BR PCNT_UNIT_2
#define PCNT_UNIT_BL PCNT_UNIT_3

// Define Encoder Pins
#define ENC_A_FR 16
#define ENC_B_FR 22
#define ENC_A_FL 34
#define ENC_B_FL 15
#define ENC_A_BR 2
#define ENC_B_BR 39
#define ENC_A_BL 26
#define ENC_B_BL 35

// void app_main(void) {
//     // Initialize encoder
//     encoder_init(PCNT_UNIT_BL, ENC_A_BL, ENC_B_BL);
    
//     while (1) {
//         int encoder_count = read_encoder(PCNT_UNIT_BL);
//         ESP_LOGI(TAG, "Front Right Encoder Count: %d", encoder_count);
//         vTaskDelay(pdMS_TO_TICKS(500));  // Log every 500ms
//     }
// }
