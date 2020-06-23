// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>
#include <string>

#include <frc/DriverStation.h>

Drivetrain Robot::drivetrain;
Elevator Robot::elevator;
Climber Robot::climber;
Intake Robot::intake;
frc::Joystick Robot::appendageStick{kAppendageStickPort};
frc::Joystick Robot::driveStick1{kDriveStick1Port};
frc::Joystick Robot::driveStick2{kDriveStick2Port};

LiveGrapher Robot::liveGrapher{kLiveGrapherPort};

Logger Robot::logger;
CsvLogger Robot::csvLogger{kCSVFile};

Robot::Robot() : PublishNode("Robot") {
    // Auton: does nothing
    dsDisplay.AddAutoMethod(
        "No-op", [] {}, [] {});
    dsDisplay.AddAutoMethod("Autoline",
                            std::bind(&AutoAutoLine::Reset, &autoLine),
                            std::bind(&AutoAutoLine::Run, &autoLine));
    dsDisplay.AddAutoMethod("Left Position Switch",
                            std::bind(&AutoLeftSwitch::Reset, &leftSwitch),
                            std::bind(&AutoLeftSwitch::Run, &leftSwitch));
    dsDisplay.AddAutoMethod("Center Position Switch",
                            std::bind(&AutoCenterSwitch::Reset, &centerSwitch),
                            std::bind(&AutoCenterSwitch::Run, &centerSwitch));
    dsDisplay.AddAutoMethod("Right Position Switch",
                            std::bind(&AutoRightSwitch::Reset, &rightSwitch),
                            std::bind(&AutoRightSwitch::Run, &rightSwitch));
    dsDisplay.AddAutoMethod("Left Position Priority",
                            std::bind(&AutoLeftPriority::Reset, &leftPriority),
                            std::bind(&AutoLeftPriority::Run, &leftPriority));
    dsDisplay.AddAutoMethod(
        "Right Position Priority",
        std::bind(&AutoRightPriority::Reset, &rightPriority),
        std::bind(&AutoRightPriority::Run, &rightPriority));
    dsDisplay.AddAutoMethod("Left Position Scale",
                            std::bind(&AutoLeftScale::Reset, &leftScale),
                            std::bind(&AutoLeftScale::Run, &leftScale));
    dsDisplay.AddAutoMethod("Center Position Scale",
                            std::bind(&AutoCenterScale::Reset, &centerScale),
                            std::bind(&AutoCenterScale::Run, &centerScale));
    dsDisplay.AddAutoMethod("Right Position Scale",
                            std::bind(&AutoRightScale::Reset, &rightScale),
                            std::bind(&AutoRightScale::Run, &rightScale));
    dsDisplay.AddAutoMethod("Left Position Double",
                            std::bind(&AutoLeftDouble::Reset, &leftDouble),
                            std::bind(&AutoLeftDouble::Run, &leftDouble));
    dsDisplay.AddAutoMethod("Right Position Double",
                            std::bind(&AutoRightDouble::Reset, &rightDouble),
                            std::bind(&AutoRightDouble::Run, &rightDouble));
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

    logger.Subscribe(*this);
    climber.Subscribe(*this);
    intake.Subscribe(*this);
    elevator.Subscribe(*this);
}

void Robot::DisabledInit() {
    drivetrain.Disable();
    // drivetrain.ResetGyro();
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
    // drivetrain.ResetGyro();
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
            ButtonPacket message{"AppendageStick", i, true};
            Publish(message);
        }
        if (appendageStick.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick", i, false};
            Publish(message);
        }
    }
}

void Robot::DisabledPeriodic() {
    // drivetrain.SetGoal(Pose(0.0, 0.0, 0.0));
}

void Robot::AutonomousPeriodic() {
    dsDisplay.ExecAutonomousPeriodic();
    // TODO: add logging of state-space controllers
    DS_PrintOut();
}

void Robot::TeleopPeriodic() {
    // drivetrain.ProcessMessage(EventType::kTimeout);
    // elevator.ProcessMessage(EventType::kTimeout);
    // climber.ProcessMessage(EventType::kTimeout);
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
