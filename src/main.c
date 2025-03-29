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

int app_main() {

    if (setup() != 0) {
        for (int jordyn = 0; jordyn < 7; ++jordyn) {
            led_flash(&robot_singleton.headlight);
        }

        ESP_LOGE(TAG, "Setup failed. Restarting");
        vTaskDelay(200);
        esp_restart();
    }
    state_t currentState;
    currentState = READY;       // Begin in READY mode

    while(currentState != END) {
        switch (currentState) {
            case RESET:
                ESP_LOGI(TAG, "Restarting...");
                vTaskDelay(200);
                esp_restart();
                break;
            case READY:
                wait_for_push_start();
                vTaskDelay(pdMS_TO_TICKS(5000));
                // wait_for_push_start();
                // wait_for_push_start();
                // currentState = START_LED_SENSE;
                currentState = FULL_SEARCH;
                break;
            case FULL_SEARCH:

                Outside_Cave_Part_1();

                Inside_Cave();
            
                Outside_Cave_Part_2();

                Outside_Cave_Part_3();

                currentState = SHIMMY;

                
                break;
            case SHIMMY:
                dc_set_speed(&robot_singleton.intakeMotor, 0);
                perform_maneuver(robot_singleton.omniMotors, ROTATE_CLOCKWISE, NULL, 20);
                for (int i = 0; i < 10; ++i) {
                    if (i % 2 == 0) {
                        servo_set_angle(&robot_singleton.armMotor, 150);
                    } else {
                        servo_set_angle(&robot_singleton.armMotor, 60);
                    }
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                perform_maneuver(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, NULL, 20);
                for (int i = 0; i < 10; ++i) {
                    if (i % 2 == 0) {
                        servo_set_angle(&robot_singleton.armMotor, 150);
                    } else {
                        servo_set_angle(&robot_singleton.armMotor, 60);
                    }
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                currentState = STOP_PROGRAM;

                break;
            case STOP_PROGRAM:
                perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
                dc_set_speed(&robot_singleton.intakeMotor, 0);
                dc_set_speed(&robot_singleton.outtakeMotor, 0);
                servo_set_angle(&robot_singleton.armMotor, 240);
                led_set_brightness(&robot_singleton.headlight, 0);
                currentState = END;
                break;
            case END:
        }
    }
    exit(0);
}

