// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

#include <CameraServer.h>
#include <DriverStation.h>
#include <Joystick.h>
#include <TimedRobot.h>
#include <Timer.h>
#include <XboxController.h>
#include <cscore.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DSDisplay/DSDisplay.hpp"
#include "LiveGrapher/LiveGrapher.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/Climber.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Elevator.hpp"
#include "Subsystems/Intake.hpp"

class Robot : public frc::TimedRobot {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

    void AutoAutoLine();
    void AutoAutoLineTimed();
    void AutoLeftPos();
    void AutoCenterPos();
    void AutoRightPos();

    void DS_PrintOut();

    static Intake intake;
    static Elevator elevator;
    static Climber climber;

private:
    std::string gameData;

    using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;

    DriveTrain robotDrive;

    frc::Joystick driveStick1{kDriveStick1Port};
    frc::Joystick driveStick2{kDriveStick2Port};
    frc::Joystick appendageStick{kAppendageStickPort};

    frc::Timer autoTimer;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{kDsPort};

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    cs::UsbCamera camera2{"Camera 2", 1};

    cs::MjpegServer server{"Server", kMjpegServerPort};

    // LiveGrapher host
    // LiveGrapher liveGrapher{kLiveGrapherPort};
};
