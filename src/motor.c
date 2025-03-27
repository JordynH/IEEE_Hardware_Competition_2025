/**
 * @file motor.c
 * @brief Motor setup and control functions
 * 
 * This file defines and implements the functions to initialize and control DC motors using the MCPWM peripheral on the ESP32.
 * This file also defines and implements functions to control a servo motor using the MCPWM peripheral.
 * 
 * @author Solomon Tolson
 * @version 1.1
 * @date 2025-02-11
 * @modified 2025-03-17
 */

#include "motor.h"
#include "encoder.h"  // Include encoder header for reading encoder values
#include "pid.h"      // Include PID header for PID control

#include "esp_log.h"

#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MAX_ANGLE 300.0         // Maximum angle in degrees
#define TAG "MOTOR"
#define FORWARD_SPEED_CONSTANT  0.66   // Feet per second
#define STRAFE_SPEED_CONSTANT 0.5      // Feet per second
#define ROTATE_SPEED_CONSTANT 42        // degrees per second

// --- PID & Encoder Definitions for Closed-Loop Movement ---
// 5203 series Yellow Jacket motors have a gear ratio of 19.2:1.
// Since the raw encoder produces ~537.7 ticks per motor revolution, the effective ticks per wheel revolution are:
#define RAW_TICKS_PER_REV 537.7
#define GEAR_RATIO 19.2
#define TICKS_PER_REV 268
#define MAX_ENCODER_VELOCITY_TICKS 1300
#define UPDATE_INTERVAL_MS 50

// Define your wheel’s diameter in feet. (For example, if the wheel is 96 mm = 3.78 inches, then ~0.315 feet.)
#define WHEEL_DIAMETER_FEET 0.3412
#define WHEEL_CIRCUMFERENCE_FEET (WHEEL_DIAMETER_FEET * 3.14159265358979)


mcpwm_timer_handle_t timers[2][3];      // Array of 3 timers in each of 2 groups
mcpwm_oper_handle_t opers[2][3];        // Array of 3 operators in each of 2 groups

void init_motor_resources() {
    for (int group = 0; group < 2; ++group) {
        for (int number = 0; number < 3; ++number) {
            mcpwm_timer_config_t timer_config = {
                .group_id = group,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = 1000000,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = 20000
            };
            mcpwm_new_timer(&timer_config, &timers[group][number]);
            mcpwm_timer_enable(timers[group][number]);
            mcpwm_timer_start_stop(timers[group][number], MCPWM_TIMER_START_NO_STOP);

            mcpwm_operator_config_t operator_config = {
                .group_id = group,
            };
            mcpwm_new_operator(&operator_config, &opers[group][number]);
        }
    }
}

void motor_control_init(motor_t *motor) {
    motor->timer = timers[motor->group_id][motor->timer_id];
    motor->oper = opers[motor->group_id][motor->oper_id];
    mcpwm_operator_connect_timer(motor->oper, motor->timer);
 
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(motor->oper, &comparator_config, &motor->comparator);
    if (motor->type == DC_MOTOR) {
        mcpwm_comparator_set_compare_value(motor->comparator, 0);
    } else if (motor->type == SERVO_MOTOR) {
        mcpwm_comparator_set_compare_value(motor->comparator, SERVO_MIN_PULSEWIDTH_US);
    }
    
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = motor->pwm_pin,
    };
    mcpwm_new_generator(motor->oper, &generator_config, &motor->gen);
    
     
    mcpwm_generator_set_action_on_timer_event(
        motor->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
 
    mcpwm_generator_set_action_on_compare_event(motor->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator, MCPWM_GEN_ACTION_LOW));
 
    // Assign PWM output pin
 
    //ESP_LOGI("MOTOR", "Initialized motor on pin %d", motor->pwm_pin);

    
}
 
void servo_set_angle(motor_t *servo, float angle) {
    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    // Map angle to pulse width
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
                            ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * (angle / SERVO_MAX_ANGLE));
 
    // Ensure the generator outputs HIGH at the start of the cycle
    mcpwm_generator_set_action_on_timer_event(
        servo->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    );
 
    if (servo->comparator == NULL) {
        mcpwm_comparator_config_t cmp_config = {
            .flags.update_cmp_on_tez = true
        };
        mcpwm_new_comparator(servo->oper, &cmp_config, &servo->comparator);
    }
 
    // Set new pulse width
    mcpwm_comparator_set_compare_value(servo->comparator, pulse_width);
 
    mcpwm_gen_compare_event_action_t cmp_event = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .action = MCPWM_GEN_ACTION_LOW,
        .comparator = servo->comparator
    };
 
    mcpwm_generator_set_action_on_compare_event(servo->gen, cmp_event);
 
    // ESP_LOGI("SERVO", "Servo angle set to %.2f degrees (Pulse width: %dus)", angle, (int)pulse_width);
}
 
 
void dc_set_speed(motor_t *motor, float speed) {
    if (speed < -100) {
        speed = -100;
    } else if (speed > 100) {
        speed = 100;
    }
 
    float min_pulse = 1050;
    float max_pulse = 1950;
    float pulse_width = (min_pulse + ((speed + 100) / 200) * (max_pulse - min_pulse));
 
    // Set comparator value dynamically
    mcpwm_comparator_set_compare_value(motor->comparator, (uint32_t)pulse_width);
 
    // ESP_LOGI("MOTOR", "Motor speed set: pin %d, pulse %dus (speed %.2f)",
            //  motor->pwm_pin, (int)pulse_width, speed);
}
 
 
void perform_maneuver(motor_t *motors, maneuver_t maneuver, float speeds[4], float speed_scalar) {
    // Ensure speed_scalar is within the range [0, 100]
    if (speed_scalar < 0) speed_scalar = 0;
    if (speed_scalar > 100) speed_scalar = 100;
 
    // Define motor positions
    motor_t *front_right = &motors[0];
    motor_t *front_left = &motors[1];
    motor_t *back_right = &motors[2];
    motor_t *back_left = &motors[3];
    double FR_ADJUSTMENT = -1 * 0.95;
    double FL_ADJUSTMENT = 1;
    double BR_ADJUSTMENT = -1 * 0.95;
    double BL_ADJUSTMENT = 1;
    switch (maneuver) {
        case FORWARD:
            dc_set_speed(front_right, 1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, 1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, 1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, 1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case BACKWARD:
            dc_set_speed(front_right, -1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, -1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, -1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, -1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case RIGHT:
            dc_set_speed(front_right, -1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, 1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, 1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, -1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case LEFT:
            dc_set_speed(front_right, 1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, -1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, -1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, 1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case FORWARD_LEFT:
            dc_set_speed(front_right, 0);
            dc_set_speed(front_left, 1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, 1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, 0);
            break;
        case FORWARD_RIGHT:
            dc_set_speed(front_right, 1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, 0);
            dc_set_speed(back_right, 0);
            dc_set_speed(back_left, 1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case BACKWARD_LEFT:
            dc_set_speed(front_right, -1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, 0);
            dc_set_speed(back_right, 0);
            dc_set_speed(back_left, -1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case BACKWARD_RIGHT:
            dc_set_speed(front_right, 0);
            dc_set_speed(front_left, -1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, -1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, 0);
            break;
        case ROTATE_COUNTERCLOCKWISE:
            dc_set_speed(front_right, 1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, -1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, 1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, -1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case ROTATE_CLOCKWISE:
            dc_set_speed(front_right, -1 * speed_scalar * FR_ADJUSTMENT);
            dc_set_speed(front_left, 1 * speed_scalar * FL_ADJUSTMENT);
            dc_set_speed(back_right, -1 * speed_scalar * BR_ADJUSTMENT);
            dc_set_speed(back_left, 1 * speed_scalar * BL_ADJUSTMENT);
            break;
        case STOP:
            dc_set_speed(front_right, 0);
            dc_set_speed(front_left, 0);
            dc_set_speed(back_right, 0);
            dc_set_speed(back_left, 0);
            break;
        case CUSTOM:
            dc_set_speed(front_right, speeds[0] * speed_scalar);
            dc_set_speed(front_left, speeds[1] * speed_scalar);
            dc_set_speed(back_right, speeds[2] * speed_scalar);
            dc_set_speed(back_left, speeds[3] * speed_scalar);
            break;
        default:
            // Handle invalid maneuver
            break;
    }
}

void outtake_dump(motor_t *outtakeMotor) {
    int64_t start_time = esp_timer_get_time();
    int64_t elapsed_time = 0;
    dc_set_speed(outtakeMotor, 25);
    while (elapsed_time < (5350000)) {
        elapsed_time = esp_timer_get_time() - start_time;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    dc_set_speed(outtakeMotor, 0);
}

void outtake_reset(motor_t *outtakeMotor) {
    int64_t start_time = esp_timer_get_time();
    int64_t elapsed_time = 0;
    dc_set_speed(outtakeMotor, -25);
    while (elapsed_time < (4850000)) {
        elapsed_time = esp_timer_get_time() - start_time;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    dc_set_speed(outtakeMotor, 0);
}

void move_distance_hardcode(motor_t *motors, maneuver_t maneuver, float speed_scalar, double feet) {
    int64_t start_time = esp_timer_get_time();
    int64_t elapsed_time = 0;
    double multiplier = 0.0;
    if (maneuver == FORWARD || maneuver == BACKWARD) {
        multiplier = FORWARD_SPEED_CONSTANT;
    } else if (maneuver == LEFT || maneuver == RIGHT) {
        multiplier = STRAFE_SPEED_CONSTANT;
    }
    int64_t target_time = 1000000 * feet / ((speed_scalar / 25)* multiplier);
    while (elapsed_time < target_time) {
        perform_maneuver(motors, maneuver, NULL, speed_scalar);
        vTaskDelay(20);
        elapsed_time = esp_timer_get_time() - start_time;
    }
    perform_maneuver(motors, STOP, NULL, 0); 
}

void rotate_angle_hardcode(motor_t *motors, maneuver_t maneuver, float speed_scalar, int degrees) {
    int64_t start_time = esp_timer_get_time();
    int64_t elapsed_time = 0;
    int64_t target_time = 1000000 * degrees / ((speed_scalar / 25) * ROTATE_SPEED_CONSTANT);
    while (elapsed_time < target_time) {
        perform_maneuver(motors, maneuver, NULL, speed_scalar);
        vTaskDelay(20);
        elapsed_time = esp_timer_get_time() - start_time;
    }
    perform_maneuver(motors, STOP, NULL, 0); 
}

//*****************************************************************************
// New PID-based Closed-Loop Movement Functions
//*****************************************************************************

// /**
//  * @brief Converts a distance in feet to the number of encoder ticks.
//  */
// static int distance_to_ticks(double distance_feet) {
//     return (int)((distance_feet / WHEEL_CIRCUMFERENCE_FEET) * TICKS_PER_REV);
// }


void move_pid_time(motor_t *motors, maneuver_t maneuver, float speed_scalar, double duration_seconds) {
    // Determine the desired angular velocity (in encoder ticks per second) based on desired speed
    double target_velocity = (speed_scalar / 100.0) * MAX_ENCODER_VELOCITY_TICKS;

    // Define the direction multipliers for each motor (FR, FL, BR, BL)
    int direction[4];

    switch (maneuver) {
        case FORWARD:
            direction[0] = 1;  direction[1] = -1;
            direction[2] = 1;  direction[3] = -1;
            break;
        case BACKWARD:
            direction[0] = -1; direction[1] = 1;
            direction[2] = -1; direction[3] = 1;
            break;
        case LEFT:  // Strafe left
            direction[0] = 1; direction[1] = 1;
            direction[2] = -1;  direction[3] = -1;
            break;
        case RIGHT: // Strafe right
            direction[0] = -1;  direction[1] = -1;
            direction[2] = 1; direction[3] = 1;
            break;
        case ROTATE_CLOCKWISE:
            direction[0] = -1; direction[1] = -1;
            direction[2] = -1; direction[3] = -1;
            break;
        case ROTATE_COUNTERCLOCKWISE:
            direction[0] = 1;  direction[1] = 1;
            direction[2] = 1;  direction[3] = 1;
            break;
        default:
            ESP_LOGE(TAG, "Invalid maneuver!");
            return;
    }

    // Initialize encoder
    init_all_encoders();
    
    // Initialize PID controllers for each motor (angle-based)
    PIDController pid_fr, pid_fl, pid_br, pid_bl;
    // pid_init(&pid_fr, 0.15, 0.05, 0.5);
    // pid_init(&pid_fl, 0.15, 0.05, 0.5);
    // pid_init(&pid_br, 0.15, 0.05, 0.5);
    // pid_init(&pid_bl, 0.15, 0.05, 0.5);

    pid_init(&pid_fr, 1.25, 0.0, 0.03);
    pid_init(&pid_fl, 1.25, 0.0, 0.03);
    pid_init(&pid_br, 1.25, 0.0, 0.03);
    pid_init(&pid_bl, 0.8, 0.0, 0.03);
    
    // Get initial encoder counts as the starting angle (position)
    int start_angle[4] = {
        read_encoder(PCNT_UNIT_0),  // Front Right
        read_encoder(PCNT_UNIT_1),  // Front Left
        read_encoder(PCNT_UNIT_2),  // Back Right
        read_encoder(PCNT_UNIT_3)   // Back Left
    };
    
    int64_t start_time = esp_timer_get_time();
    
    while ((esp_timer_get_time() - start_time) < (duration_seconds * 1e6)) {
        int64_t now = esp_timer_get_time();
        double elapsed = (now - start_time) / 1e6;

        //ESP_LOGI(TAG, "Target Angle: %d", (int) target_angle);

        // Compute the evolving target angles for each motor
        double target_angle = target_velocity * elapsed;
        int target[4] = {
            start_angle[0] + direction[0] * target_angle,  // Front Right
            start_angle[1] + direction[1] * target_angle,  // Front Left
            start_angle[2] + direction[2] * target_angle,  // Back Right
            start_angle[3] + direction[3] * target_angle   // Back Left
        };
        
         // Read current positions
         int current[4] = {
            read_encoder(PCNT_UNIT_0),
            read_encoder(PCNT_UNIT_1),
            read_encoder(PCNT_UNIT_2),
            read_encoder(PCNT_UNIT_3)
        };

        // Compute PID corrections based on position error
        int output[4] = {
            pid_compute(&pid_fr, target[0], current[0]),
            pid_compute(&pid_fl, target[1], current[1]),
            pid_compute(&pid_br, target[2], current[2]),
            pid_compute(&pid_bl, target[3], current[3])
        };

        //ESP_LOGI(TAG, "Read Angle: %d", (int) current_fr);
        
        // Apply the PID outputs (dc_set_speed clamps internally)
        for (int i = 0; i < 4; i++) {
            dc_set_speed(&motors[i], -output[i]);  // Negative sign ensures correct direction
        }
        
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
    
    perform_maneuver(motors, STOP, NULL, 0);
}

void move_pid_distance(motor_t *motors, maneuver_t maneuver, float speed_scalar, double distance_feet) {
    // Base speed measurement: At speed_scalar = 15, robot moves at 9 inches/sec (0.75 ft/sec)
    double base_speed_scalar = 15.0;
    double base_speed_ft_per_sec = 9.4 / 12.0;  // Convert 9 in/sec to ft/sec

    // Scale speed proportionally: new speed = base_speed * (speed_scalar / base_speed_scalar)
    double actual_speed_ft_per_sec = base_speed_ft_per_sec * (speed_scalar / base_speed_scalar);

    // Avoid division by zero if speed scalar is too low
    if (actual_speed_ft_per_sec <= 0) {
        // ESP_LOGE("MOVE_WRAPPER", "Invalid speed scalar: %.2f. Must be greater than 0.", speed_scalar);
        return;
    }

    // Compute duration required to travel the given distance
    double duration_seconds = distance_feet / actual_speed_ft_per_sec;

    // ESP_LOGI("MOVE_WRAPPER", "Moving %.2f feet at speed_scalar %.2f for %.2f seconds", 
            //  distance_feet, speed_scalar, duration_seconds);

    // Call the movement function
    move_pid_time(motors, maneuver, speed_scalar, duration_seconds);
}

