// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

#include <frc/DriverStation.h>

Drivetrain Robot::drivetrain;
Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;
frc::Joystick Robot::appendageStick{kAppendageStickPort};
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

LiveGrapher Robot::liveGrapher{kLiveGrapherPort};

Logger Robot::logger;
CsvLogger Robot::csvLogger{kCSVFile};

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
}

void Robot::DisabledInit() {
    drivetrain.Disable();
    drivetrain.ResetGyro();
    drivetrain.ResetEncoders();
    drivetrain.Reset();
    intake.SetMotors(MotorState::kIdle);
    elevator.Disable();
    elevator.Reset();
    elevator.SetGoal(elevator.GetHeight());
}

void Robot::AutonomousInit() {
    std::cout << "AutoInit" << std::endl;
    drivetrain.ResetEncoders();
    drivetrain.ResetGyro();
    drivetrain.Reset();
    elevator.ResetEncoder();
    elevator.Reset();
    intake.Deploy();
    climber.LockPawl();

    dsDisplay.ExecAutonomousInit();
    logger.Log(
        LogEvent("AUTON INITIALIZED: " + dsDisplay.GetAutonomousMode() + " " +
                     frc::DriverStation::GetInstance().GetGameSpecificMessage(),
                 LogEvent::VERBOSE_INFO));
}

void Robot::TeleopInit() {
    drivetrain.Disable();
    elevator.Disable();
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

void Robot::DisabledPeriodic() { drivetrain.SetGoal(Pose(0.0, 0.0, 0.0)); }

void Robot::AutonomousPeriodic() {
    dsDisplay.ExecAutonomousPeriodic();
    // TODO: add logging of state-space controllers
    DS_PrintOut();
}

void Robot::TeleopPeriodic() {
    if (driveStick1.GetRawButton(1)) {
        drivetrain.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        drivetrain.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }
    // drivetrain.PostEvent(EventType::kTimeout);
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

void Robot::DS_PrintOut() {
    /*if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData(
            (drivetrain.GetLeftRate() + drivetrain.GetRightRate()) / 2,
            "Average Velocity");
        liveGrapher.GraphData(drivetrain.GetAngularRate(), "Angle Rate");
        static double prevVel = 0.0;
        static double curVel = 0.0;
        curVel = drivetrain.GetAngularRate();
        liveGrapher.GraphData((curVel - prevVel) / .005, "Angle Accel");
        prevVel = curVel;
        liveGrapher.ResetInterval();
        */
    /*logger.Log(
        LogEvent("Elevator Position: " + std::to_string(elevator.GetHeight()),
                 LogEvent::VERBOSE_DEBUG));*/
    elevator.Debug();
    // std::cout << drivetrain.GetLeftDisplacement() << "Left, Right" <<
    // drivetrain.GetRightDisplacement() << std::endl;
    // std::cout << drivetrain.GetAngle() << std::endl;
    // std::cout << elevator.GetHeight() << std::endl;
    // std::cout << "Version 1.5" << std::endl; // To ensure a
    // successful(butchered) upload
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
