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

#include <stdio.h>
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_secondary.h"
#include "led.h"
#include "math.h"


#define TAG "MAIN"
/**
 * 
 */
typedef struct {
    led_t headlight;

    motor_t frontLeft;
    motor_t frontRight;
    motor_t backRight;
    motor_t backLeft;
    motor_t intakeMotor;
    motor_t outtakeMotor;
    motor_t armMotor;

    motor_t *omniMotors;    /* An array of the four maneuver DC motors*/
} robot_t;
static robot_t robot_singleton;

typedef enum {
    RESET,              // Restart the ESP and run the initialization sequence again
    READY,              // Do nothing and wait for further instructions
    START_LED_SENSE,    // Wait for the start LED at the beginning of the competition
    MOVE_CONTAINERS,    // Move the cosmic shipping containers into their correct launch pads
    COLLECT_OUTSIDE,    // Roam around and collect the asteroids outside of the cave
    COLLECT_INSIDE,     // Roam around and collect the asteroids inside of the cave
    UNLOAD,             // Unload the collected asteroids into the cosmic shipping containers
    SHIMMY,             // Celebratory shimmy
    STOP_PROGRAM,       // Stop, drop, and roll (except don't drop or roll, just stop) exit the FSM
    END
} state_t;
static state_t currentState;

int64_t start_time_us;

/* Command Key
    Px Commands = Switch to Pipeline x    
*/

void switch_pipeline(int new_pipeline) {
    char message[5];
    sprintf(message, "P%d", new_pipeline);
    send_message(message);
    while (get_pID() != (double)new_pipeline) {
        send_message(message);
        // vTaskDelay(5);
    }
}

void full_motor_init() {
    init_motor_resources();

    robot_singleton.omniMotors = malloc(sizeof(motor_t) * 4);

    motor_t frontLeft =     { .pwm_pin = 17,    .group_id = 0, .timer_id = 0, .oper_id = 0, .type = DC_MOTOR };
    motor_t frontRight =    { .pwm_pin = 21,    .group_id = 0, .timer_id = 0, .oper_id = 0, .type = DC_MOTOR };
    motor_t backLeft =      { .pwm_pin = 25,    .group_id = 0, .timer_id = 1, .oper_id = 1, .type = DC_MOTOR };
    motor_t backRight =     { .pwm_pin = 32,    .group_id = 0, .timer_id = 1, .oper_id = 1, .type = DC_MOTOR };
    motor_t intakeMotor =   { .pwm_pin = 33,    .group_id = 0, .timer_id = 2, .oper_id = 2, .type = DC_MOTOR };
    motor_t outtakeMotor =  { .pwm_pin = 4,     .group_id = 0, .timer_id = 2, .oper_id = 2, .type = DC_MOTOR };
    motor_t armMotor =      { .pwm_pin = 27,    .group_id = 1, .timer_id = 0, .oper_id = 0, .type = SERVO_MOTOR };

    motor_control_init(&frontLeft);
    motor_control_init(&frontRight);
    motor_control_init(&backRight);
    motor_control_init(&backLeft);
    motor_control_init(&intakeMotor);
    motor_control_init(&outtakeMotor);
    motor_control_init(&armMotor);

    robot_singleton.armMotor = armMotor;
    robot_singleton.frontLeft = frontLeft;
    robot_singleton.frontRight = frontRight;
    robot_singleton.backLeft = backLeft;
    robot_singleton.backRight = backRight;
    robot_singleton.intakeMotor = intakeMotor;
    robot_singleton.outtakeMotor = outtakeMotor;
    robot_singleton.armMotor = armMotor;
    robot_singleton.omniMotors[0] = frontRight;
    robot_singleton.omniMotors[1] = frontLeft;
    robot_singleton.omniMotors[2] = backRight;
    robot_singleton.omniMotors[3] = backLeft;

    dc_set_speed(&robot_singleton.intakeMotor, 0);
    dc_set_speed(&robot_singleton.outtakeMotor, 0);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
    servo_set_angle(&robot_singleton.armMotor, 90);
}

void aprilTag_main(int desired_fid, double ta_target) {
    switch_pipeline(6);
    int done = 0;

    double dy_threshold = 1;
    double tx_threshold = 5;
    double tx_epsilon = 10;
    double ta_epsilon = 0.01;

    while (!done) {
        while (get_v() == 0) { // FIXME: Check accuracy of hard-coded path and potentially 
                            // make this a while loop with a maneuver.
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        int aligned = 0;
        while (!aligned) {

            double tx = 0.0;
            double tx_tmp = 0.0;

            tx_tmp = get_fiducial_tx();
            if (fabs(tx_tmp) > 0.00001) {
                tx = tx_tmp;
            }
            double bottom_left[2];
            double bottom_right[2];
            double dy = 0.0;
            get_point_at_index(0, bottom_left);
            get_point_at_index(1, bottom_right);
            if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
                dy = bottom_right[1] - bottom_left[1];
            }
            // --- STRAFE until centered ---
            while (fabs(tx) > tx_threshold) {
                if (tx < -tx_threshold) {
                    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 25);
                } else if (tx > tx_threshold) {
                    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 25);
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                // Refresh tx reading
                tx_tmp = get_fiducial_tx();
                if (fabs(tx_tmp) > 0.00001) {
                    tx = tx_tmp;
                }
                ESP_LOGI("MAIN", "tx: %f" , fabs(tx));
            }
            perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);

            ESP_LOGI(TAG, "I did it! I got past the tx threshold! Here's my final tx value: %f" , fabs(tx));

            // --- ROTATE until epsilon ---
            get_point_at_index(0, bottom_left);
            get_point_at_index(1, bottom_right);
            if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
                dy = bottom_right[1] - bottom_left[1];
            }
            
            //&& (fabs(tx) < tx_epsilon)
            if ((dy < (-1 * dy_threshold)) && (fabs(tx) < tx_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, NULL, 15);
            } else if ((dy > dy_threshold) && (fabs(tx) < tx_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, ROTATE_CLOCKWISE, NULL, 15);
            }

            // Rotate while BOTH:
            // Not aligned (dy > threshold)
            // Still centered (tx < epsilon)
            while ((fabs(dy) > dy_threshold) && (fabs(tx) < tx_epsilon)) {

                vTaskDelay(pdMS_TO_TICKS(20));

                // Update dy and tx
                get_point_at_index(0, bottom_left);
                get_point_at_index(1, bottom_right);
                if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
                    dy = bottom_right[1] - bottom_left[1];
                }
                ESP_LOGI(TAG, "jordyn:");
                tx_tmp = get_fiducial_tx();
                if (fabs(tx_tmp) > 0.00001) {
                    tx = tx_tmp;
                }
                ESP_LOGI(TAG, "dy value: %f" , fabs(dy));
                ESP_LOGI(TAG, "Here's the heap size: %u", (unsigned int)esp_get_free_heap_size());
            }

            ESP_LOGI(TAG, "I did it! I got past the dy threshold! Here's my final dy value: %f" , fabs(dy));

            // Stop strafing when either:
            // - dy is small enough (aligned)
            // - tx is too large (not centered anymore)
            perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);

            // Refresh for aligned check
            tx_tmp = get_fiducial_tx();
            if (fabs(tx_tmp) > 0.00001) {
                tx = tx_tmp;
            }
            get_point_at_index(0, bottom_left);
            get_point_at_index(1, bottom_right);
            if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
                dy = bottom_right[1] - bottom_left[1];
            }

            // Exit if both alignment (dy) and centering (tx) are good
            if ((fabs(tx) <= tx_threshold) && (fabs(dy) <= dy_threshold)) {
                aligned = 1;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // --- DISTANCE PHASE (TA Control) ---
        int distance_done = 0;
        double ta = 0.0;
        double ta_temp = 0.0;
        while (!distance_done) {
            ta_temp = get_fiducial_ta();
            if (ta_temp > 0.00001) {
                ta = ta_temp;
                ESP_LOGI("bananaman", "namananab %f" , ta);
            }
            // ESP_LOGI("JORDYN", "(THIS IS SPAR)TA = %f" , ta);

            if (ta < (ta_target - ta_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 18);
                // while (get_fiducial_ta() < (ta_target - ta_epsilon)) {
                //     // ESP_LOGI("TAG", "Whiskey TAngo = %f" , ta);
                //     vTaskDelay(pdMS_TO_TICKS(20));
                // }
                // perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
            } else if (ta > (ta_target + ta_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, BACKWARD, NULL, 18);
                // while (get_fiducial_ta() > (ta_target + ta_epsilon)) {
                //     // ESP_LOGI("quaker", "i cant graduate because i dont know waht a gant chart is = %f" , ta);
                //     vTaskDelay(pdMS_TO_TICKS(20));
                // }
                // perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
            } else {
                distance_done = 1;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }

        done = 1; // finished
    }

    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0); // HERE
}


void wiring_test_sequence() {

    /* headlights */
    for (int i = 0; i < 100; ++i) {
        int brightness = i;
        led_set_brightness(&robot_singleton.headlight, brightness);
        ESP_LOGI("MAIN", "Set brightness: %d" , brightness);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    led_flash(&robot_singleton.headlight);

    /* front right */
    dc_set_speed(&robot_singleton.frontRight, 25);
    vTaskDelay(200);
    dc_set_speed(&robot_singleton.frontRight, 0);
    vTaskDelay(100);

    led_flash(&robot_singleton.headlight);

    /* front left */
    dc_set_speed(&robot_singleton.frontLeft, 25);
    vTaskDelay(200);
    dc_set_speed(&robot_singleton.frontLeft, 0);
    vTaskDelay(100);

    led_flash(&robot_singleton.headlight);

    /* back right */
    dc_set_speed(&robot_singleton.backRight, 25);
    vTaskDelay(200);
    dc_set_speed(&robot_singleton.backRight, 0);
    vTaskDelay(100);

    led_flash(&robot_singleton.headlight);

    /* back left */
    dc_set_speed(&robot_singleton.backLeft, 25);
    vTaskDelay(200);
    dc_set_speed(&robot_singleton.backLeft, 0);
    vTaskDelay(100);

    led_flash(&robot_singleton.headlight);

    /* servo */
    servo_set_angle(&robot_singleton.armMotor, 0);
    for (int i = 0; i < 300; i += 25) {
        servo_set_angle(&robot_singleton.armMotor, i);
        vTaskDelay(50);
    }
    servo_set_angle(&robot_singleton.armMotor, 150);

    led_flash(&robot_singleton.headlight);


    /* intake */
    dc_set_speed(&robot_singleton.intakeMotor, -100);
    vTaskDelay(200);
    dc_set_speed(&robot_singleton.intakeMotor, 0);
    vTaskDelay(100);

    led_flash(&robot_singleton.headlight);
    
    /* outtake */
    outtake_dump(&robot_singleton.outtakeMotor);
    vTaskDelay(200);
    outtake_reset(&robot_singleton.outtakeMotor);

    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
}



/**
 * @brief This function will initialize all peripherals and subsystems.
 * 
 * @details This function will setup and initialize the following:
 *      - All motors
 *          - 4 DC motors for primary maneuvers
 *          - 1 DC motor for the outtake system
 *          - 1 DC motor for the intake system
 *          - 1 SERVO motor for the arm
 *      - SPI communication with the Raspberry Pi
 *      - Front-facing LED headlights
 * 
 * @return int 0 if successful, -1 if not
 */
int setup() {
    /* 1. LED Initializatio Sequence */

    led_init(&robot_singleton.headlight);
    led_set_brightness(&robot_singleton.headlight, 50);

    led_flash(&robot_singleton.headlight);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 2. Motor Initialization Sequence */
    full_motor_init();

    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    vTaskDelay(pdMS_TO_TICKS(500));


    /* 3. RPI SPI Communication Initialization Sequence */
    spi_secondary_init();
    
    char* received = "";
    ESP_LOGI(TAG, "Waiting for Communication Initialization Confirmation");
    while((strcmp(received, "INITIALIZATION_MESSAGE"))) {
        send_message("INITIALIZATION_MESSAGE");
        // ESP_LOGI(TAG, "message = %s", get_message());
        received = get_message();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGI(TAG, "communication established");
    send_message("communication established");
    //vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Waiting for Pipeline Switch");
    switch_pipeline(1);
    
    
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);

    return 0;
}


void power_test_sequence() {
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    dc_set_speed(&robot_singleton.intakeMotor, -100);
    for (int i = 3; i < 10; ++i) {
        perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, (float)(i*10));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    servo_set_angle(&robot_singleton.armMotor, 170);
    outtake_dump(&robot_singleton.outtakeMotor);
    outtake_reset(&robot_singleton.outtakeMotor);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
    dc_set_speed(&robot_singleton.intakeMotor, 0);

}

void demo() {
    dc_set_speed(&robot_singleton.intakeMotor, -70);

    perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 25);
    vTaskDelay(pdMS_TO_TICKS(5000));
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    perform_maneuver(robot_singleton.omniMotors, BACKWARD, NULL, 25);
    vTaskDelay(pdMS_TO_TICKS(6000));
    dc_set_speed(&robot_singleton.intakeMotor, 0);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
    outtake_dump(&robot_singleton.outtakeMotor);
    vTaskDelay(pdMS_TO_TICKS(1500));
    outtake_reset(&robot_singleton.outtakeMotor);
    vTaskDelay(pdMS_TO_TICKS(10000));

}

void predetermined_test() {
    // move_distance(robot_singleton.omniMotors, FORWARD, 25, 10);
    // vTaskDelay(pdMS_TO_TICKS(400));
    // move_distance(robot_singleton.omniMotors, BACKWARD, 25, 10);
    // vTaskDelay(pdMS_TO_TICKS(400));
    // rotate_angle(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 25, 270);
    // vTaskDelay(pdMS_TO_TICKS(400));
    // rotate_angle(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 25, 270);
    // vTaskDelay(pdMS_TO_TICKS(400));
    // move_distance(robot_singleton.omniMotors, LEFT, 25, 10);
    // vTaskDelay(pdMS_TO_TICKS(400));
    // move_distance(robot_singleton.omniMotors, RIGHT, 25, 10);
    // vTaskDelay(pdMS_TO_TICKS(400));

    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);

    // move_distance_pid_angle(robot_singleton.omniMotors, FORWARD, 15, 5);
    // vTaskDelay(pdMS_TO_TICKS(500));

    // move_distance_pid_angle(robot_singleton.omniMotors, RIGHT, 15, 5);
    // vTaskDelay(pdMS_TO_TICKS(500));

    // move_distance_pid_angle(robot_singleton.omniMotors, BACKWARD, 15, 5);
    // vTaskDelay(pdMS_TO_TICKS(500));

    // move_distance_pid_angle(robot_singleton.omniMotors, LEFT, 15, 5);
    // vTaskDelay(pdMS_TO_TICKS(500));

    // rotate_angle(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 50, 90);
    // vTaskDelay(pdMS_TO_TICKS(500));

    // rotate_angle(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 50, 90);
    
}

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
    // demo();
    // dc_set_speed(&robot_singleton. outtakeMotor, -20);
    //wiring_test_sequence();

    //vTaskDelay(pdMS_TO_TICKS(1000));

    // power_test_sequence();
    // led_set_brightness(&robot_singleton.headlight, 100);
    // perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 25);
    // move_distance(robot_singleton.omniMotors, FORWARD, 25, 10);
    // outtake_dump(&robot_singleton.outtakeMotor);
    // perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 25);
    // move_distance(robot_singleton.omniMotors, LEFT, 25, 8);
    // perform_maneuver(robot_singleton.omniMotors, ROTATE_CLOCKWISE, NULL, 25);

    //predetermined_test();
    ESP_LOGI("MAIN", "got here");
    while (1) {
        vTaskDelay(100);
    }
    
    // vTaskDelay(200);
    /*all motor test*/
    // perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 25);
    // dc_set_speed(&robot_singleton.intakeMotor, -100);
    // dc_set_speed(&robot_singleton. outtakeMotor, -20);
    
    

    // dc_set_speed(&robot_singleton.outtakeMotor, -25);
    // vTaskDelay(500);
    // dc_set_speed(&robot_singleton.outtakeMotor, 0);
    // vTaskDelay(100);

    // outtake_dump(&robot_singleton.outtakeMotor);
    // vTaskDelay(300);
    // outtake_reset(&robot_singleton.outtakeMotor);

    // outtake_reset(&robot_singleton.outtakeMotor);
    // vTaskDelay(500);
    // outtake_dump(&robot_singleton.outtakeMotor);
    // for (int i = 0; i < 300; i+=20) {
    //     servo_set_angle(&robot_singleton.armMotor, i);
    //     vTaskDelay(100);
    // }
    
    vTaskDelay(5000);
    dc_set_speed(&robot_singleton.intakeMotor, 0);
    dc_set_speed(&robot_singleton.outtakeMotor, 0);
    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);

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

