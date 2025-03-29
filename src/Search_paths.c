#include "Search_paths.h"

double TA_FAR = 0.05;
double TA_MID = 0.08;
double TA_CLOSE = 0.12;


// Outside the Cave - Part 1: Initial S Sweep
void Outside_Cave_Part_1() {
    dc_set_speed(&robot_singleton.intakeMotor, -75);

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.45);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.25);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2);
    move_pid_time(robot_singleton.omniMotors, LEFT, 10, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 1.4);

    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.85);

    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.85);
    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);

    dump_in_geo();

    monitor_stack_usage();

    move_pid_time(robot_singleton.omniMotors, LEFT, 15, 0.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.75);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);

    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 22);

    aprilTag_main(-1, 0.08);

    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.9);

    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);
    
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
}

// Inside the Cave: Sweep for Astral Material
void Inside_Cave() {
    // led_set_brightness(NULL, 75);

    // Initial forward move.
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.2);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2);
    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 22);
    aprilTag_main(-1, 0.08);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1);

    // --- First outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);

    // Inner loop, d == 1
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);

    // --- Second outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);

    // Inner loop, d == 1:
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);

    // --- Third outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);

    // Inner loop, d == 1:
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.7);

    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);

    aprilTag_main(-1, 0.08);

    // Move backwards to allow a 180° turn.
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.25);

    
    
    // // Final alignment: rotate 180° and exit.
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    // // Rotate right and exit cave sideways.
    // move_pid_time(robot_singleton.omniMotors, LEFT, 15, 3);

    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 2);

    led_set_brightness(NULL, 50);

}

// Outside the Cave - Part 2: Dump Geodinium and Move Nebulite Bin
void Outside_Cave_Part_2() {
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.5);

    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 2.75);

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.5);

    dump_in_geo();
}

// Outside the Cave - Part 3: Re-sweep, Dump, and Move Geodinium Bin
void Outside_Cave_Part_3() {
    // Prepare for dump alignment
    // RotateRight(90);
    // SlideLeftUntilSouthAprilTagInView();
    // CenterCameraOnSouthAprilTag();
    // SlideLeftToPredeterminedTXForDump();
    // MoveForwardToPredeterminedTAForDump();
    // Rotate(180);

    // // Back up to align with bin and wall
    // MoveBackward(1.5);
    // StartOuttakeMotor();
    // TimedDelayForDump();

    // // Fine bin positioning
    // SlideLeftInches(7);   // Slide 7 inches (width of bin)
    // BackUpAgainstWall();
    // StartServoArmMotor(110);  // Turn servo arm by 110 degrees
    // MoveForward(0.5);

    // // Re-align using north April tag for final dump
    // SlideLeftUntilNorthAprilTagInView();
    // CenterCameraOnNorthAprilTag();
    // SlideLeftToPredeterminedTXForAlignment();
    // MoveForwardToPredeterminedTAForAlignment();
    // RotateLeft(90);

    // // Final alignment using line following
    // StartLineFollowingScript();
    // CenterOnWhiteLinesFacingWestWall();
    // MoveForwardUntilEndOfWhiteCaveEntranceTape();

    // // Rendezvous alignment using west April tag
    // VerifyWestAprilTagInView();
    // ReadWestAprilTag();
    // MoveForwardToPredeterminedTAForRendezvous();
    // RotateLeft(90);

    // // Final line following for rendezvous pad approach
    // StartLineFollowingScript();
    // CenterBetweenStartBoxAndRendezvousPad();
    // AdjustMovementBasedOnWestAprilTag();
}

void Short_Search_All() {
        dc_set_speed(&robot_singleton.intakeMotor, -75);

        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.35);

        move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 1.25);

        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 3.15);

        move_pid_time(robot_singleton.omniMotors, LEFT, 15, 2.75);

        move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.75);

        move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 2.85);

        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.25);

        move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 2.25);

        move_pid_time(robot_singleton.omniMotors, LEFT, 15, 1.25);

        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.75);

        move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.75);

        move_pid_time(robot_singleton.omniMotors, LEFT, 15, 1.25);

        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.75);

        perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);

        dump_in_geo();

        move_pid_time(robot_singleton.omniMotors, LEFT, 15, 0.5);
        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.75);
        move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);

        //aprilTag_main(-1, 0.08);

        //Inside_Cave();

        dc_set_speed(&robot_singleton.intakeMotor, 0);


    // while (1) {
    //     move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.5);

    //     vTaskDelay(500);

    //     move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 2.5);
        
    // }
}
