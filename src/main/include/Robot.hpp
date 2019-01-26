// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cscore.h>

#include <string>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "autonomousmodes/AutoAutoLine.hpp"
#include "autonomousmodes/AutoCenterScale.hpp"
#include "autonomousmodes/AutoCenterSwitch.hpp"
#include "autonomousmodes/AutoLeftDouble.hpp"
#include "autonomousmodes/AutoLeftPriority.hpp"
#include "autonomousmodes/AutoLeftScale.hpp"
#include "autonomousmodes/AutoLeftSwitch.hpp"
#include "autonomousmodes/AutoRightDouble.hpp"
#include "autonomousmodes/AutoRightPriority.hpp"
#include "autonomousmodes/AutoRightScale.hpp"
#include "autonomousmodes/AutoRightSwitch.hpp"
#include "dsdisplay/DSDisplay.hpp"
#include "es/Service.hpp"
#include "livegrapher/LiveGrapher.hpp"
#include "logging/CsvLogger.hpp"
#include "logging/LogConsoleSink.hpp"
#include "logging/LogFileSink.hpp"
#include "logging/Logger.hpp"
#include "subsystems/CANTalonGroup.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/Intake.hpp"

class Robot : public frc::TimedRobot, public Service {
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

    void HandleEvent(Event event) override;

    void DS_PrintOut();

    static Drivetrain drivetrain;
    static Intake intake;
    static Elevator elevator;
    static Climber climber;
    static frc::Joystick appendageStick;
    static frc::Joystick driveStick1;
    static frc::Joystick driveStick2;

    // LiveGrapher host
    static LiveGrapher liveGrapher;

    // Logging
    static Logger logger;

    static CsvLogger csvLogger;

private:
    AutoAutoLine autoLine;
    AutoCenterScale centerScale;
    AutoCenterSwitch centerSwitch;
    AutoLeftDouble leftDouble;
    AutoLeftPriority leftPriority;
    AutoLeftScale leftScale;
    AutoLeftSwitch leftSwitch;
    AutoRightDouble rightDouble;
    AutoRightPriority rightPriority;
    AutoRightScale rightScale;
    AutoRightSwitch rightSwitch;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{kDsPort};

    // Logging Sinks
    LogFileSink fileSink{"/home/lvuser/Robot.log"};
    LogConsoleSink consoleSink;

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};

    // cs::MjpegServer server{"Server", kMjpegServerPort};
};
