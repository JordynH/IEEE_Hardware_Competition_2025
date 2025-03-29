#include "Search_paths.h"

double TA_FAR = 0.05;
double TA_MID = 0.08;
double TA_CLOSE = 0.12;


// Outside the Cave - Part 1: Initial S Sweep
void Outside_Cave_Part_1() {
    dc_set_speed(&robot_singleton.intakeMotor, -75);

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.25);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.8);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.42);
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


    /* ---------- */


    /* ---------- */

    dump_in_geo();

    // monitor_stack_usage();

    move_pid_time(robot_singleton.omniMotors, LEFT, 15, 0.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.75);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);

    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 22);

    aprilTag_main(-1, 0.06);

    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.7);

    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
}

// Inside the Cave: Sweep for Astral Material
void Inside_Cave() {
    // led_set_brightness(NULL, 75);

    // Initial forward move.
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.2);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.5);
    perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 22);
    aprilTag_main(-1, 0.08);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1);

    // --- First outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);

    // Inner loop, d == 1
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);

    // --- Second outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);

    // Inner loop, d == 1:
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);

    // --- Third outer loop iteration ---
    // Inner loop, d == 0:
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);

    // Inner loop, d == 1:
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.42);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.8);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 1.7);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);

    // Advance to new AprilTag depth.
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.4);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.7);

    perform_maneuver(robot_singleton.omniMotors, BACKWARD, NULL, 22);

    aprilTag_main(-1, 0.10);

    // Move backwards to allow a 180° turn.
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.6);

    
    
    // // Final alignment: rotate 180° and exit.
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    // // Rotate right and exit cave sideways.
    // move_pid_time(robot_singleton.omniMotors, LEFT, 15, 3);

    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.4);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 0.5);
    move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 4);

    led_set_brightness(NULL, 50);

}

// Outside the Cave - Part 2: Dump Geodinium and Move Nebulite Bin
void Outside_Cave_Part_2() {

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.9);

    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);
    dump_in_geo();
    // move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.7);


}

// Outside the Cave - Part 3: Re-sweep, Dump, and Move Geodinium Bin
void Outside_Cave_Part_3() {


    
    // move_pid_time(robot_singleton.omniMotors, LEFT, 15, 1);
    // // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.75);
    // // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);
    // // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.5);

    // // perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 22);
    
    // // aprilTag_main(-1, 0.08);

    // // move_pid_time(robot_singleton.omniMotors, RIGHT, 7.5, 0.5);                 /* move away from the bucket */
    // // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);      /* spin 90° */
    // // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.45);      /* spin 90° */
    // move_pid_time(robot_singleton.omniMotors, BACKWARD, 7.5, 2);                /* back into the wall */
    // move_pid_time(robot_singleton.omniMotors, RIGHT, 7.5, 0.75);                  /* strafe into the bucket */
    // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 7.5, 0.7);

    // servo_set_angle(&robot_singleton.armMotor, 150);                         /* lower the arm to grab the bucket */
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // move_pid_time(robot_singleton.omniMotors, LEFT, 10, 0.7);
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 2.5);                /* move to the center of the arena */
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 10, 2);  /* rotate to face west */
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 4);

    // perform_maneuver(robot_singleton.omniMotors, RIGHT, NULL, 22);

    // aprilTag_main(-1, 0.05);
    // int fid = get_fiducial_fID();
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 10, 2);  /* rotate to face south */
    // if (fid < 2) {
    //     move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 1); // TUNE time
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 240);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    //     move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 1); // TUNE time
    // } else if (fid > 2) {
    //     move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 1); // TUNE time
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 240);
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    //     move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 1);
    // } else {
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 240);
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    // }
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.42);
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2);
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.4);
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);

    // perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);

    move_pid_time(robot_singleton.omniMotors, LEFT, 15, 0.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 3.5);
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 0.4);
    move_pid_time(robot_singleton.omniMotors, LEFT, 15, 3.5);

    // aprilTag_main(-1, 0.08);

    // move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 1);
    // move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.7);
    // move_pid_time(robot_singleton.omniMotors, LEFT, 15, 3);
    // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.8);
    // move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 2);

    // move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 1);      // TUNE
    // servo_set_angle(&robot_singleton.armMotor, 150);

    // move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 1.5);

    // move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 10, 2);

    // perform_maneuver(robot_singleton.omniMotors, FORWARD, NULL, 15);

    // aprilTag_main(-1, 0.05);
    // fid = get_fiducial_fID();
    // move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 10, 2);  /* rotate to face south */
    // if (fid < 2) {
    //     move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 1); // TUNE time
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 60);
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    //     move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 1); // TUNE time
    // } else if (fid > 2) {
    //     move_pid_time(robot_singleton.omniMotors, BACKWARD, 10, 1); // TUNE time
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 60);
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    //     move_pid_time(robot_singleton.omniMotors, FORWARD, 10, 1);
    // } else {
    //     move_pid_time(robot_singleton.omniMotors, RIGHT, 10, 2);
    //     servo_set_angle(&robot_singleton.armMotor, 60);
    //     move_pid_time(robot_singleton.omniMotors, LEFT, 10, 2);
    // }
    

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
