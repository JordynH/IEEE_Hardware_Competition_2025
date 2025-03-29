

#include "main_helpers.h"

#define TAG "MAIN HELPER"

robot_t robot_singleton;

int setup() {
    /* 1. LED Initialization Sequence */
    led_init(&robot_singleton.headlight);
    led_set_brightness(&robot_singleton.headlight, 50);

    led_flash(&robot_singleton.headlight);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 2. Push Button, Limit Switch, LED Start Initialization */
    setup_limit_switch();
    setup_push_start();
    // setup_start_led();

    /* 3. Motor Initialization Sequence */
    full_motor_init();

    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    vTaskDelay(pdMS_TO_TICKS(500));


    /* 4. RPI SPI Communication Initialization Sequence */
    spi_secondary_init();
    
    char* received = "";
    ESP_LOGI(TAG, "Waiting for Communication Initialization Confirmation");
    while(get_pID() < 0 && (strcmp(received, "INITIALIZATION_MESSAGE"))) {
        send_message("INITIALIZATION_MESSAGE");
        // ESP_LOGI(TAG, "message = %s", get_message());
        received = get_message();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGI(TAG, "communication established");
    send_message("communication established");
    //vTaskDelay(pdMS_TO_TICKS(500));
    
    // ESP_LOGI(TAG, "Waiting for Pipeline Switch");
    switch_pipeline(6);
    
    
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);

    return 0;
}

void switch_pipeline(int new_pipeline) {
    char message[5];
    sprintf(message, "P%d", new_pipeline);
    send_message(message);
    led_flash(&robot_singleton.headlight);
    while (get_pID() != (double)new_pipeline) {
        send_message(message);
        vTaskDelay(5);
    }
    led_flash(&robot_singleton.headlight);
    led_flash(&robot_singleton.headlight);

}

void full_motor_init() {
    init_motor_resources();
    init_all_encoders();

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
    servo_set_angle(&robot_singleton.armMotor, 60);
    outtake_reset(&robot_singleton.outtakeMotor);
}

void aprilTag_main(int desired_fid, double ta_target) {
    // switch_pipeline(6);
    ESP_LOGI(TAG, "1");
    int done = 0;

    double dy_threshold = 3;
    double tx_threshold = 3;
    double tx_epsilon = 10;
    double ta_epsilon = 0.01;
    double ta_temp = 0.0;
    double ta = 0.0;
    double tx = 0.0;
    double tx_tmp = 0.0;
    double bottom_right[2];
    double bottom_left[2];
    double dy = 0.0;
    ESP_LOGI(TAG, "2");
    while (!done) {
        ESP_LOGI(TAG, "3");
        while (get_v() == 0) { // FIXME: Check accuracy of hard-coded path and potentially 
                            // make this a while loop with a maneuver.

            led_flash(&robot_singleton.headlight);
            vTaskDelay(pdMS_TO_TICKS(20));
            ESP_LOGI(TAG, "4");
        }
        ESP_LOGI(TAG, "5");
        int aligned = 0;
        while (!aligned) {
            monitor_stack_usage();
            ESP_LOGI(TAG, "6");
            tx = 0.0;
            tx_tmp = 0.0;

            tx_tmp = get_fiducial_tx();
            if (fabs(tx_tmp) > 0.00001) {
                tx = tx_tmp;
            }
            dy = 0.0;
            get_point_at_index(0, bottom_left);
            get_point_at_index(1, bottom_right);
            if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
                dy = bottom_right[1] - bottom_left[1];
            }
            // --- STRAFE until centered ---
            while (fabs(tx) > tx_threshold) {
                ESP_LOGI(TAG, "7");
                monitor_stack_usage();
                ta_temp = get_fiducial_ta();
                if (ta_temp > 0.00001) {
                    ta = ta_temp;
                    ESP_LOGI("bananaman", "namananab %f" , ta);
                }
                if (tx < -tx_threshold) {
                    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, (23 * (1 - ta)));
                } else if (tx > tx_threshold) {
                    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, (23 * (1 - ta)));
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                // Refresh tx reading
                tx_tmp = get_fiducial_tx();
                if (fabs(tx_tmp) > 0.00001) {
                    tx = tx_tmp;
                }
                ESP_LOGI("MAIN", "tx: %f" , fabs(tx));
                ESP_LOGI(TAG, "8");
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
                perform_maneuver(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, NULL, 18);
            } else if ((dy > dy_threshold) && (fabs(tx) < tx_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, ROTATE_CLOCKWISE, NULL, 18);
            }

            // Rotate while BOTH:
            // Not aligned (dy > threshold)
            // Still centered (tx < epsilon)
            while ((fabs(dy) > dy_threshold) && (fabs(tx) < tx_epsilon)) {
                monitor_stack_usage();

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
        if (ta_target < 0) {
            distance_done = 1;
        }
        
        ta = 0.0;
        ta_temp = 0.0;
        while (!distance_done) {
            monitor_stack_usage();

            ta_temp = get_fiducial_ta();
            if (ta_temp > 0.00001) {
                ta = ta_temp;
                ESP_LOGI("bananaman", "namananab %f" , ta);
            }
            // ESP_LOGI("JORDYN", "(THIS IS SPAR)TA = %f" , ta);

            if (ta < (ta_target - ta_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, (18 * (1 - ta)));
                // while (get_fiducial_ta() < (ta_target - ta_epsilon)) {
                //     // ESP_LOGI("TAG", "Whiskey TAngo = %f" , ta);
                //     vTaskDelay(pdMS_TO_TICKS(20));
                // }
                // perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0);
            } else if (ta > (ta_target + ta_epsilon)) {
                perform_maneuver(robot_singleton.omniMotors, BACKWARD, NULL, (18 * (1 - ta)));
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
        tx_tmp = get_fiducial_tx();
        tx = 0.0;
        if (fabs(tx_tmp) > 0.00001) {
            tx = tx_tmp;
        }
        dy = 0.0;
        get_point_at_index(0, bottom_left);
        get_point_at_index(1, bottom_right);
        if (fabs(bottom_right[1] - bottom_left[1]) > 0.00001) {
            dy = bottom_right[1] - bottom_left[1];
        }
        if ((tx < tx_threshold) && (fabs(dy) < dy_threshold)) {
            done = 1;
        }
    }

    perform_maneuver(robot_singleton.omniMotors, STOP, NULL, 0); // HERE
    monitor_stack_usage();
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

void predetermined_test() {
    dc_set_speed(&robot_singleton.intakeMotor, -80);

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.25);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.6);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);

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

void dump_in_geo() {
    monitor_stack_usage();
    aprilTag_main(-1, 0.1);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);
    move_pid_time(robot_singleton.omniMotors, RIGHT, 7.5, 1.75);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 7.5, 0.85);
    outtake_dump(&robot_singleton.outtakeMotor);
    vTaskDelay(pdMS_TO_TICKS(800));
    outtake_reset(&robot_singleton.outtakeMotor);
}

// void setup_start_led() {
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_DISABLE,      // disable interrupts
//         .mode = GPIO_MODE_INPUT,             // set as input mode
//         .pin_bit_mask = (1ULL << GPIO_NUM_14),    // bit mask for WAIT_PIN
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .pull_up_en   = GPIO_PULLUP_DISABLE
//     };
//     gpio_config(&io_conf);
// }

void setup_limit_switch() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // disable interrupts
        .mode = GPIO_MODE_INPUT,             // set as input mode
        .pin_bit_mask = (1ULL << GPIO_NUM_14),    // bit mask for WAIT_PIN
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);
}

void setup_push_start()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // disable interrupts
        .mode = GPIO_MODE_INPUT,             // set as input mode
        .pin_bit_mask = (1ULL << GPIO_NUM_22),  // bit mask for INPUT_PIN2
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE     // enable internal pull-up
    };
    gpio_config(&io_conf);
}

void wait_for_start_led()
{
    ESP_LOGI(TAG, "Waiting for LED sense to go HIGH...");
    while (gpio_get_level(GPIO_NUM_12) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay to avoid busy looping
    }
    ESP_LOGI(TAG, "LED sense is HIGH");
}

void wait_for_push_start()
{
    ESP_LOGI(TAG, "Waiting for push start to go LOW...");
    while (gpio_get_level(GPIO_NUM_22) != 0) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Delay to avoid busy looping
    }
    ESP_LOGI(TAG, "Push start is LOW");
}