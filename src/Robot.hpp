// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Joystick.h>
#include <TimedRobot.h>
#include <Timer.h>
#include <cscore.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DSDisplay/DSDisplay.hpp"
#include "LiveGrapher/LiveGrapher.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/DriveTrain.hpp"

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

    void DS_PrintOut();

private:
    using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;

    DriveTrain robotDrive;

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};

    frc::Timer autoTimer;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{k_dsPort};

    // LiveGrapher host
    // LiveGrapher liveGrapher{k_liveGrapherPort};
};
