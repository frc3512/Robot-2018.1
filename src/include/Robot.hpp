// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <sys/stat.h>

#include <array>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <tuple>
#include <utility>

#include <Joystick.h>
#include <TimedRobot.h>
#include <cscore.h>

#include "AutonomousModes/AutoAutoLine.hpp"
#include "AutonomousModes/AutoCenterScale.hpp"
#include "AutonomousModes/AutoCenterSwitch.hpp"
#include "AutonomousModes/AutoLeftDouble.hpp"
#include "AutonomousModes/AutoLeftPriority.hpp"
#include "AutonomousModes/AutoLeftScale.hpp"
#include "AutonomousModes/AutoLeftSwitch.hpp"
#include "AutonomousModes/AutoRightDouble.hpp"
#include "AutonomousModes/AutoRightPriority.hpp"
#include "AutonomousModes/AutoRightScale.hpp"
#include "AutonomousModes/AutoRightSwitch.hpp"
#include "Constants.hpp"
#include "DSDisplay/DSDisplay.hpp"
#include "ES/Service.hpp"
#include "LiveGrapher/LiveGrapher.hpp"
#include "Logging/LogConsoleSink.hpp"
#include "Logging/LogFileSink.hpp"
#include "Logging/Logger.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/Climber.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Elevator.hpp"
#include "Subsystems/Intake.hpp"

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

    static std::string GetFileCreationTime(std::string filePath);
    static std::pair<uint64_t, uint64_t> GetDataUsage();

    void HandleEvent(Event event) override;

    void DS_PrintOut();

    std::string version;

    static DriveTrain robotDrive;
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
