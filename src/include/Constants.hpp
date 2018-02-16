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
constexpr int kDsPort = 5800;

// LiveGrapher host port
constexpr int kLiveGrapherPort = 3513;

// MJPEG server port
constexpr int kMjpegServerPort = 1180;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int kLeftDriveMasterID = 1;
constexpr int kLeftDriveSlaveID = 2;
constexpr int kRightDriveMasterID = 3;
constexpr int kRightDriveSlaveID = 4;
// Distance per Pulse
constexpr double kLeftDpP = 169.0 / 11915.0;
constexpr double kRightDpP = 169.0 / 11839.3;

// DriveTrain position PID, Extra //'s mean practice PID values
constexpr double kDriveMaxSpeed = 24000;  // in/sec
constexpr double kPosP = 0.00;
constexpr double kPosI = 0.00;
constexpr double kPosD = 0.00;

// DriveTrain angle PID
constexpr double kRotateMaxSpeed = 320;
constexpr double kAngleP = 0.00;
constexpr double kAngleI = 0.00;
constexpr double kAngleD = 0.00;

// Climber Solenoid ports
constexpr int kAlignmentArmsPort = 3;
constexpr int kSetupForwardPort = 4;
constexpr int kSetupReversePort = 5;

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;

/*
 * Intake
 */

// Solenoid Ports
constexpr int kIntakeClawPort = 1;
constexpr int kIntakeArmPort = 2;

// Talon IDs
constexpr int kIntakeLeftID = 5;
constexpr int kIntakeRightID = 6;

/*
 * Elevator
 */

// Elevator GearBox ID
constexpr int kElevatorMasterID = 7;
constexpr int kElevatorSlaveID = 8;

// Distance per Pulse
constexpr double kElevatorDpP = 0.00142230843;

// Elevator PID
constexpr double kElevatorP = 0.0;
constexpr double kElevatorI = 0.0;
constexpr double kElevatorD = 0.0;
constexpr double kElevatorControllerPeriod = 1.0;

// Elevator Setpoints
constexpr double kFloorHeight = 0.0;
constexpr double kSwitchHeight = 12.0;
constexpr double kScaleHeight = 60.0;
constexpr double kClimbHeight = 80.0;

// Hall Effect Sensor Port
constexpr int kElevatorForwardHallPort = 1;
constexpr int kElevatorReverseHallPort = 0;

// Physical Robot Constants
constexpr int kRobotLength = 0;

// Event Queue Size
constexpr int kEventQueueSize = 8;
