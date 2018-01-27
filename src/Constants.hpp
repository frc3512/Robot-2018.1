// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of subsystem constants:
 * > Motor IDs
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

// DS port
constexpr int k_dsPort = 5800;

// LiveGrapher host port
constexpr int k_liveGrapherPort = 3513;

// MJPEG server port
constexpr int k_mjpegServerPort = 1180;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int k_driveStick1Port = 0;
constexpr int k_driveStick2Port = 1;
constexpr int k_grabberStickPort = 2;

// Joystick axis deadband range
constexpr double k_joystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int k_leftDriveMasterID = 1;
constexpr int k_leftDriveSlaveID = 2;
constexpr int k_rightDriveMasterID = 3;
constexpr int k_rightDriveSlaveID = 4;

// DriveTrain position PID, Extra //'s mean practice PID values
constexpr double k_driveMaxSpeed = 24000;  // in/sec
constexpr double k_posP = 0.00;            // 0.07
constexpr double k_posI = 0.00;            // 0.00
constexpr double k_posD = 0.00;            // 0.08

// DriveTrain angle PID
constexpr double k_rotateMaxSpeed = 320;
constexpr double k_angleP = 0.75;  // 0.75
constexpr double k_angleI = 0.00;  // 0.00
constexpr double k_angleD = 0.05;  // 0.05

// CheesyDrive constants
constexpr double k_lowGearSensitive = 0.75;
constexpr double k_turnNonLinearity = 1.0;
constexpr double k_inertiaDampen = 2.5;
constexpr double k_inertiaHighTurn = 3.0;
constexpr double k_inertiaLowTurn = 3.0;
