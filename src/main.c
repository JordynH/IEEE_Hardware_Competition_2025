/**
 * @file main.c
 * @brief This will be the main program to control the ESP.
 * 
 * @details This program will integrate all of the hardware peripherals in addition to controlling the primary state machine.
 * 
 * @version 1.0
 * @date 2025-01-28
 * @modified 2025-03-17
 */

#include "main_helpers.h"

#define TAG "MAIN"

int64_t start_time_us;

/* Command Key
    Px Commands = Switch to Pipeline x    
*/

int app_main() {
    if (setup() != 0) {
        for (int jordyn = 0; jordyn < 7; ++jordyn) {
            led_flash(&robot_singleton.headlight);
        }

        ESP_LOGE(TAG, "Setup failed. Restarting");
        vTaskDelay(200);
        esp_restart();
    }

    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 25);
    aprilTag_main(-1, 0.08);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 25);

    ESP_LOGI("MAIN", "got here");
    while (1) {
        vTaskDelay(100);
    }
    
    vTaskDelay(5000);
    dc_set_speed(&robot_singleton.intakeMotor, 0);
    dc_set_speed(&robot_singleton.outtakeMotor, 0);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);

    state_t currentState;
    currentState = READY;       // Begin in LED SENSE mode

    while(currentState != END) {
        switch (currentState) {
            case RESET:
                ESP_LOGI(TAG, "Restarting...");
                vTaskDelay(200);
                esp_restart();
                break;
            case READY:
                break;
            case START_LED_SENSE:
                send_message("P0");
                while ((strcmp(get_message(), "led found"))) {
                    vTaskDelay(10);
                }
                currentState = COLLECT_OUTSIDE;
                start_time_us = esp_timer_get_time();
                break;
            case MOVE_CONTAINERS:
            case COLLECT_OUTSIDE:
            case COLLECT_INSIDE:
            case UNLOAD:
            case SHIMMY:
            case STOP_PROGRAM:
                perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
                dc_set_speed(&robot_singleton.intakeMotor, 0);
                dc_set_speed(&robot_singleton.outtakeMotor, 0);
                servo_set_angle(&robot_singleton.armMotor, 90);
                currentState = END;
                break;
            case END:
        }
    }
    exit(0);
}

