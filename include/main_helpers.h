

#ifndef MAIN_HELPERS_H
#define MAIN_HELPERS_H

#include <stdio.h>
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_secondary.h"
#include "led.h"
#include "math.h"

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
extern robot_t robot_singleton;

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
// extern state_t currentState;

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
int setup();

void switch_pipeline(int new_pipeline);

void full_motor_init();

void aprilTag_main(int desired_fid, double ta_target);

void predetermined_test();

void demo();

void power_test_sequence();

#endif // MAIN_HELPERS_H