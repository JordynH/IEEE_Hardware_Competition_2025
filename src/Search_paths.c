#include "Search_paths.h"

double TA_FAR = 0.05;
double TA_MID = 0.08;
double TA_CLOSE = 0.12;


// Outside the Cave - Part 1: Initial S Sweep
void Outside_Cave_Part_1() {
    dc_set_speed(&robot_singleton.intakeMotor, -80);

    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.35);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.4);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.9);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);

    // Center on North April Tag
    aprilTag_main(-1, 0.08);

    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2.0);
    perform_maneuver(robot_singleton.omniMotors, LEFT, NULL, 22);

    dump_in_geo();

    move_pid_time(robot_singleton.omniMotors, LEFT, 15, 0.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.8);
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.75);
    
    // Cetner on South April Tag
    aprilTag_main(-1, 0.05);

    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.6);
    move_pid_time(robot_singleton.omniMotors, ROTATE_COUNTERCLOCKWISE, 15, 1.5);
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2);
}

// Inside the Cave: Sweep for Astral Material
void Inside_Cave() {
    move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 2);

    led_set_brightness(NULL, 75);

    // Initial approach: find first east AprilTag at furthest threshold
    aprilTag_main(-1, TA_FAR);

    // Nested loop: three proximity thresholds x two slide directions
    double thresholds[3] = {TA_FAR, TA_MID, TA_CLOSE};
    maneuver_t slide_dirs[2] = {RIGHT, LEFT};
    maneuver_t rotate_dirs[2] = {ROTATE_COUNTERCLOCKWISE, ROTATE_CLOCKWISE};

    for (int t = 1; t < 3; t++) {
        for (int d = 0; d < 2; d++) {
            // Rotate into corridor
            move_pid_time(robot_singleton.omniMotors, rotate_dirs[d], 15, 1.5);
            // Move forward 1.5 ft
            move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 1.5);
            if (d == 1) {
                // Rotate back to main cave
                move_pid_time(robot_singleton.omniMotors, rotate_dirs[0], 15, 1.5);
            } else {
                // Rotate back to main cave
                move_pid_time(robot_singleton.omniMotors, rotate_dirs[1], 15, 1.5);
            }
            // slide back to center
            move_pid_time(robot_singleton.omniMotors, slide_dirs[d], 15, 1.5);
        }
        // move forwardto new april tag depth
        move_pid_time(robot_singleton.omniMotors, FORWARD, 15, 0.5);
        // Slide into next East April tag distance
        aprilTag_main(-1, thresholds[t]);
    }

    // Move backwards so can make 180 turn
    move_pid_time(robot_singleton.omniMotors, BACKWARD, 15, 0.4);

    // Center robot with east april tag at ta_far
    aprilTag_main(-1, TA_FAR);

    // Final alignment: rotate 180° and exit
    move_pid_time(robot_singleton.omniMotors, ROTATE_CLOCKWISE, 15, 2.9);
    // Rotate right and exit cave sideways
    move_pid_time(robot_singleton.omniMotors, RIGHT, 15, 1.5);

    // Cetner on South April Tag
    aprilTag_main(-1,0.2);

    led_set_brightness(NULL, 50);

}

// Outside the Cave - Part 2: Dump Geodinium and Move Nebulite Bin
void Outside_Cave_Part_2() {
    // Position for collecting game cubes
    // RotateLeft(90);
    // SlideRightUntilSouthAprilTagInView();
    // CenterCameraOnSouthAprilTag();
    // MoveToPredeterminedTXForCollection();
    // MoveForwardToPredeterminedTA();
    // Rotate(180);
    
    // // Prepare for bucket push
    // MoveForward(2.5);
    // SlideLeftUntilNorthAprilTagInView();
    // CenterCameraOnNorthAprilTag();
    // SlideLeftToPredeterminedTXForBucketPush();
    // MoveForwardToPredeterminedTAForBucketPush();

    // // Execute bucket movement
    // SlideBucketLeft(3);
    // SlideBack(0.5);
    // RotateLeft(90);

    // // Re-align with west April tag
    // SlideLeftUntilWestAprilTagInView();
    // CenterCameraOnWestAprilTag();
    // MoveBackwardUntilProximityForTurn();
    // Rotate(180);

    // // Begin final alignment using line following
    // MoveForwardUntilLineFollowingActivated();
    // CenterOnWhiteLinesFacingCave();
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
