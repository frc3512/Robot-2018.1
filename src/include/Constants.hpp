// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of constants:
 * > Motor IDs
 * > Solenoid Ports
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

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
constexpr double kLeftDpP = 108.0 / 8142.66;
constexpr double kRightDpP = 108.0 / 8187.66;

// DriveTrain position PID
constexpr double kPosP = 0.05;
constexpr double kPosI = 0.00;
constexpr double kPosD = 0.024;

// DriveTrain angle PID
constexpr double kAngleP = 0.05;
constexpr double kAngleI = 0.00;
constexpr double kAngleD = 0.012;

// Physical Robot Constants
constexpr int kRobotLength = 31.5;

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;

/*
 * Intake
 */

// Talon IDs
constexpr int kIntakeLeftID = 5;
constexpr int kIntakeRightID = 6;

// Solenoid Ports
constexpr int kIntakeClawPort = 1;
constexpr int kIntakeArmPort = 2;

/*
 * Elevator
 */

// Elevator GearBox ID
constexpr int kElevatorMasterID = 7;
constexpr int kElevatorSlaveID = 8;

// Hall Effect Sensor Port
constexpr int kElevatorBottomHallPort = 1;

// Distance per Pulse
constexpr double kElevatorDpP = 0.00142230843;

// Elevator PID
constexpr double kElevatorP = 0.09;
constexpr double kElevatorI = 0.004;
constexpr double kElevatorD = 0.0;
constexpr double kElevatorControllerPeriod = 0.02;
constexpr double kGravityFeedForward = -0.01;

// Elevator Setpoints
constexpr double kFloorHeight = 73.0;  // Makes sure it hits bottom
constexpr double kSecondBlockHeight = -10.0;
constexpr double kSwitchHeight = -36.0;
constexpr double kScaleHeight = -60.0;
constexpr double kClimbHeight = -72.0;

/*
 * Climber
 */

// Climber Solenoid ports
constexpr int kAlignmentArmsPort = 3;
constexpr int kSetupForwardPort = 4;
constexpr int kSetupReversePort = 5;

/*
 * Miscellaneous
 */

// Game Data Constants
constexpr int kFriendlySwitch = 0;
constexpr int kScale = 1;
constexpr int kEnemySwitch = 2;

// Event Queue Size
constexpr int kEventQueueSize = 8;

// DS port
constexpr int kDsPort = 5800;

// LiveGrapher host port
constexpr int kLiveGrapherPort = 3513;

// MJPEG server port
constexpr int kMjpegServerPort = 1180;
