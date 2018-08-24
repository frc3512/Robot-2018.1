// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units.h>

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
 * Math and Conversions
 */

constexpr double kPi = 3.1415926535897932;
constexpr double kGravity = 9.80665;

template <class T>
constexpr T deg2rad(const T& value) {
    return value * kPi / 180;
}

template <class T>
constexpr T rad2deg(const T& value) {
    return value * 180 / kPi;
}

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

constexpr int kLeftEncoderA = 6;
constexpr int kLeftEncoderB = 5;
constexpr int kRightEncoderA = 8;
constexpr int kRightEncoderB = 7;

// Controller constants
constexpr double kB = 1.0;
constexpr double kZeta = 1.0;
constexpr double kDt = 0.00505;
constexpr auto kDt_s = 0.00505_s;

// Drive trapezoid profile constants
constexpr auto kRobotMaxV = 5.461_mps;              // m/sec
constexpr auto kRobotMaxA = 1.0922_mps_sq;          // 3.0;           // sec
constexpr auto kRobotMaxRotateRate = 4.52_mps;      // rad/sec
constexpr auto kRobotMaxRotateAccel = 4.52_mps_sq;  // sec

// Physical Robot Constants
constexpr double kWheelbaseWidth = 0.6096;  // 24.0;
constexpr double kRobotLength = 0.9398;     // 37.0;  // Approximate
constexpr double kRobotWidth = 0.8382;      // 33.0;   // Approximate
constexpr double kMaxControlVoltage = 12;
constexpr double kWheelRadius = 0.0746125;  // 2.9375;  // 2.947
constexpr double kDriveGearRatio = 1.0 / 1.0;

// Distance per Pulse
constexpr double kDpP = (2.0 * kPi * kWheelRadius) * kDriveGearRatio / 2048.0;

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

constexpr auto kElevatorMaxV = 0.7703947_mps;  // m/sec
constexpr double kElevatorTimeToMaxV = 2.56;   // sec
constexpr auto kElevatorMaxA = 0.3009354_mps_sq;

// Elevator GearBox ID
constexpr int kElevatorMasterID = 7;
constexpr int kElevatorSlaveID = 8;

// Hall Effect Sensor Port
constexpr int kElevatorBottomHallPort = 1;

// Distance per Pulse
constexpr double kElevatorDpP = 0.770382 / 22589.66;  // 30.33 / 22589.66;

// Elevator Setpoints
constexpr double kFloorHeight = 1.8542;  // 73.0;  // Makes sure it hits bottom
constexpr double kSecondBlockHeight = -0.254;  // -10.0;
constexpr double kSwitchHeight = -0.9144;      // -36.0;
constexpr double kScaleHeight = -1.8161;       // -71.5;  // -67.0
constexpr double kClimbHeight = -1.8161;       // -71.5;  // 74.0

// Elevator Physical Constants
constexpr double kCarriageMass = 6.803886;     // kilograms
constexpr double kDrumRadius = 0.02762679089;  // meters
constexpr double kElevatorGearRatio = 42.0 / 12.0 * 40.0 / 14.0;
constexpr double kNumMotors = 2.0;
constexpr double kStallTorque = 2.42 * kNumMotors;    // N-m
constexpr double kStallCurrent = 133.0 * kNumMotors;  // amps
constexpr double kFreeSpeed = 5310.0;                 // no load rpm
constexpr double kFreeCurrent = 2.7 * kNumMotors;     // amps
constexpr double kResistance = 12.0 / kStallCurrent;  // resistance of motor
constexpr double kKt = kStallTorque / kStallCurrent;  // torque constant

/*
 * Climber
 */

// Climber Solenoid ports
constexpr int kPawlPort = 3;
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

// CSV Logging File
constexpr const char* kCSVFile = "/home/lvuser/CSVFile.log";

// Offset generated by the exchange forcing our robot not to be in the center
constexpr double kExchangeOffset = kRobotWidth / 2.0 - 0.3048;
