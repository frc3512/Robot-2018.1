// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

#include <DriverStation.h>

DriveTrain Robot::robotDrive;
Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;
frc::Joystick Robot::appendageStick{kAppendageStickPort};
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

LiveGrapher Robot::liveGrapher{kLiveGrapherPort};

Logger Robot::logger;

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {}, [] {});
    dsDisplay.AddAutoMethod(
        "Autoline", std::bind(&AutoAutoLine::Reset, &autoLine),
        std::bind(&AutoAutoLine::PostEvent, &autoLine, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Switch", std::bind(&AutoLeftSwitch::Reset, &leftSwitch),
        std::bind(&AutoLeftSwitch::PostEvent, &leftSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Center Position Switch",
        std::bind(&AutoCenterSwitch::Reset, &centerSwitch),
        std::bind(&AutoCenterSwitch::PostEvent, &centerSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Switch",
        std::bind(&AutoRightSwitch::Reset, &rightSwitch),
        std::bind(&AutoRightSwitch::PostEvent, &rightSwitch, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Priority",
        std::bind(&AutoLeftPriority::Reset, &leftPriority),
        std::bind(&AutoLeftPriority::PostEvent, &leftPriority, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Priority",
        std::bind(&AutoRightPriority::Reset, &rightPriority),
        std::bind(&AutoRightPriority::PostEvent, &rightPriority, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Scale", std::bind(&AutoLeftScale::Reset, &leftScale),
        std::bind(&AutoLeftScale::PostEvent, &leftScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Center Position Scale",
        std::bind(&AutoCenterScale::Reset, &centerScale),
        std::bind(&AutoCenterScale::PostEvent, &centerScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Scale", std::bind(&AutoRightScale::Reset, &rightScale),
        std::bind(&AutoRightScale::PostEvent, &rightScale, kTimeout));
    dsDisplay.AddAutoMethod(
        "Left Position Double", std::bind(&AutoLeftDouble::Reset, &leftDouble),
        std::bind(&AutoLeftDouble::PostEvent, &leftDouble, kTimeout));
    dsDisplay.AddAutoMethod(
        "Right Position Double",
        std::bind(&AutoRightDouble::Reset, &rightDouble),
        std::bind(&AutoRightDouble::PostEvent, &rightDouble, kTimeout));
    
    version = GetFileCreationTime("/home/lvuser/FRCUserProgram");

    // server.SetSource(camera1);

    // camera1.SetVideoMode(PixelFormat.kYUYV, 320, 240, 30)
    camera1.SetResolution(160, 120);
    camera1.SetFPS(15);

    // camera2.SetResolution(640, 480);
    // camera2.SetFPS(30);

    fileSink.SetVerbosityLevels(LogEvent::VERBOSE_ALL);
    consoleSink.SetVerbosityLevels(LogEvent::VERBOSE_WARN);
    logger.AddLogSink(fileSink);
    logger.AddLogSink(consoleSink);

    DS_PrintOut();
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    robotDrive.ResetGyro();
    robotDrive.ResetEncoders();
    intake.SetMotors(MotorState::kIdle);
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
}

void Robot::AutonomousInit() {
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    elevator.ResetEncoder();
    intake.Deploy();
    climber.LockPawl();

    dsDisplay.ExecAutonomousInit();
    logger.Log(
        LogEvent("AUTON INITIALIZED: " + dsDisplay.GetAutonomousMode() + " " +
                     frc::DriverStation::GetInstance().GetGameSpecificMessage(),
                 LogEvent::VERBOSE_INFO));
}

void Robot::TeleopInit() {
    robotDrive.StopClosedLoop();
    elevator.StopClosedLoop();
    intake.Deploy();
    intake.Close();
    climber.LockPawl();
    logger.Log(LogEvent("TELEOP INITIALIZED", LogEvent::VERBOSE_INFO));
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    if (!elevator.GetBottomHallEffect()) {
        elevator.ResetEncoder();
    }

    for (int i = 2; i <= 12; i++) {
        if (appendageStick.GetRawButtonPressed(i)) {
            Event event{EventType::kButtonPressed, i};
            Robot::PostEvent(event);
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
        if (appendageStick.GetRawButtonReleased(i)) {
            Event event{EventType::kButtonReleased, i};
            Robot::PostEvent(event);
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
    }
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() {
    dsDisplay.ExecAutonomousPeriodic();
    logger.Log(LogEvent(
        "Pos Goal: " + std::to_string(robotDrive.GetPositionGoal()) +
            " Pos: " + std::to_string(robotDrive.GetPosition()) +
            " At Goal?: " + std::to_string(robotDrive.AtPositionGoal()),
        LogEvent::VERBOSE_DEBUG));
    logger.Log(
        LogEvent("Angle Goal: " + std::to_string(robotDrive.GetAngleGoal()) +
                     " Angle: " + std::to_string(robotDrive.GetAngle()) +
                     " At Goal?: " + std::to_string(robotDrive.AtAngleGoal()),
                 LogEvent::VERBOSE_DEBUG));
    DS_PrintOut();
}

void Robot::TeleopPeriodic() {
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }
    // robotDrive.PostEvent(EventType::kTimeout);
    elevator.PostEvent(EventType::kTimeout);
    climber.PostEvent(EventType::kTimeout);
}

void Robot::HandleEvent(Event event) {
    /*if (event == Event{kButtonPressed, 11}) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }*/
}


std::string Robot::GetFileCreationTime(std::string filePath) {
    struct stat attrib;
    stat(filePath.c_str(), &attrib);
    std::string ret(21, '\0');
    std::strftime(&ret[0], 21, "%F %R:%S", std::localtime(&(attrib.st_ctime)));
    return ret;
}

std::pair<uint64_t, uint64_t> Robot::GetDataUsage() {
    static int oldTransmittedData = 0;
    static int oldRecievedData = 0;

    auto oldTime = std::chrono::steady_clock::now();

    auto currentTime = std::chrono::steady_clock::now();
    std::ifstream in("/proc/net/dev");
    std::string line;
    std::getline(in, line);

    std::regex reg2(
        "eth0:\\s+(\\d+)\\s+(\\d+)\\s+(\\d+)\\s+(\\d+)\\s+(\\d+)\\s+(\\d+)\\s+("
        "\\d+)\\s+"
        "(\\d+)\\s+(\\d+)");  // We only need bytes and 9 is all we need to
                              // bytes transmitted
    std::smatch match;
    std::regex_match(line, match, reg2);
    uint64_t newRecievedData = std::stoi(match[1]);
    uint64_t newTransmittedData = std::stoi(match[9]);

    int recievedDataRate =
        (newRecievedData - oldRecievedData) /
        std::chrono::duration_cast<std::chrono::seconds>(currentTime - oldTime)
            .count();
    int transmittedDataRate =
        (newTransmittedData - oldTransmittedData) /
        std::chrono::duration_cast<std::chrono::seconds>(currentTime - oldTime)
            .count();

    oldRecievedData = newRecievedData;
    oldTransmittedData = newTransmittedData;
    oldTime = currentTime;

    double recievedData =
        recievedDataRate / 1000000;  // conversion to mbps rather than bps
    double transmittedData = transmittedDataRate / 1000000;
    return std::make_pair(recievedData, transmittedData);
}

void Robot::DS_PrintOut() {
    /*if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData(
            (robotDrive.GetLeftRate() + robotDrive.GetRightRate()) / 2,
            "Average Velocity");
        liveGrapher.GraphData(robotDrive.GetAngularRate(), "Angle Rate");
        static double prevVel = 0.0;
        static double curVel = 0.0;
        curVel = robotDrive.GetAngularRate();
        liveGrapher.GraphData((curVel - prevVel) / .005, "Angle Accel");
        prevVel = curVel;
        liveGrapher.ResetInterval();
        */
    logger.Log(
        LogEvent("Elevator Position: " + std::to_string(elevator.GetHeight()),
                 LogEvent::VERBOSE_DEBUG));
    robotDrive.Debug();
    // std::cout << robotDrive.GetLeftDisplacement() << "Left, Right" << robotDrive.GetRightDisplacement() << std::endl;
    // std::cout << robotDrive.GetAngle() << std::endl;
    // std::cout << elevator.GetHeight() << std::endl;
    // std::cout << "Version 1.5" << std::endl; // To ensure a
    // successful(butchered) upload
    
    auto usage = GetDataUsage();

    std::cout << usage.first << ", " << usage.second << std::endl;
    dsDisplay.Clear();

    dsDisplay.AddData("ENCODER_LEFT", robotDrive.GetLeftDisplacement());
    dsDisplay.AddData("ENCODER_RIGHT", robotDrive.GetRightDisplacement());
    dsDisplay.AddData("GYRO_VAL", robotDrive.GetAngle());
    /*dsDisplay.AddData("PAWL_ENGAGED",
                      climber.GetPawl());  // todo: add the function
    dsDisplay.AddData("LOW_GEAR",
                      climber.IsLowGear());  // todo: add the function */
    dsDisplay.AddData("VERSION", version);

    dsDisplay.SendToDS();
}

START_ROBOT_CLASS(Robot)
