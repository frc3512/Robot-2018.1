// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>

Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});
    dsDisplay.AddAutoMethod("Autoline Timed",
                            std::bind(&Robot::AutoAutoLineTimed, this));
    dsDisplay.AddAutoMethod("Autoline", std::bind(&Robot::AutoAutoLine, this));
    dsDisplay.AddAutoMethod("Left Position",
                            std::bind(&Robot::AutoLeftPos, this));
    dsDisplay.AddAutoMethod("Center Position",
                            std::bind(&Robot::AutoCenterPos, this));
    dsDisplay.AddAutoMethod("Right Position",
                            std::bind(&Robot::AutoRightPos, this));

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);

    camera2.SetResolution(640, 480);
    camera2.SetFPS(30);

    server.SetSource(camera1);
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    robotDrive.ResetGyro();
    robotDrive.ResetEncoders();
}

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    elevator.ResetEncoder();
    intake.Deploy();
}

void Robot::TeleopInit() {
    // robotDrive.StopClosedLoop();
    robotDrive.StartClosedLoop();
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    DS_PrintOut();

    if (!elevator.GetForwardHallEffect()) {
        elevator.ResetEncoder();
    }

    for (int i = 1; i < 12; i++) {
        if (appendageStick.GetRawButtonPressed(i)) {
            Event event{EventType::kButtonPressed, i};
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
    }
}

void Robot::DisabledPeriodic() {
    if (driveStick1.GetRawButtonPressed(12)) {
        robotDrive.ResetEncoders();
    }
}

void Robot::AutonomousPeriodic() {
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    dsDisplay.ExecAutonomous();
}

void Robot::TeleopPeriodic() {
    robotDrive.SetPositionReference(driveStick1.GetY() * -50.0);
    robotDrive.SetAngleReference(driveStick1.GetX() * 0.0);
    /*    // Drive Stick Controls
if (driveStick1.GetRawButton(1)) {
    robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                     driveStick2.GetRawButton(2));
} else {
    robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                     driveStick2.GetRawButton(2));
}*/

    // Intake Controls
    if (appendageStick.GetRawButtonPressed(3)) {
        if (intake.IsOpen()) {
            intake.Close();
        } else {
            intake.Open();
        }
    }
    if (appendageStick.GetRawButtonPressed(5)) {
        if (intake.IsDeployed()) {
            intake.Stow();
        } else {
            intake.Deploy();
        }
    }
    if (appendageStick.GetRawButtonPressed(4)) {
        intake.SetMotors(MotorState::kIntake);
    }
    if (appendageStick.GetRawButtonPressed(6)) {
        intake.SetMotors(MotorState::kOuttake);
    }
    if (appendageStick.GetRawButtonReleased(4) ||
        appendageStick.GetRawButtonReleased(6)) {
        intake.SetMotors(MotorState::kIdle);
    }

    // Elevator Controls
    elevator.SetVelocity(appendageStick.GetY());

    if (appendageStick.GetRawButton(7)) {
        elevator.SetHeightReference(kFloorHeight);
        elevator.SetHeightReference(kFloorHeight);
    }
    if (appendageStick.GetRawButton(8)) {
        elevator.SetHeightReference(kSwitchHeight);
    }
    if (appendageStick.GetRawButton(9)) {
        elevator.SetHeightReference(kScaleHeight);
    }
    if (appendageStick.GetRawButton(10)) {
        elevator.SetHeightReference(kClimbHeight);
    }

    if (appendageStick.GetRawButtonPressed(11)) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }
}

void Robot::DS_PrintOut() {
    robotDrive.Debug();
    if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData(robotDrive.GetAngleReference(),
                              "Angle Reference");
        liveGrapher.GraphData(robotDrive.GetAngle(), "Angle");
        // liveGrapher.GraphData(robotDrive.GetPosReference(), "Position
        // Reference");  liveGrapher.GraphData(robotDrive.GetPosition(),
        // "Position");
        liveGrapher.ResetInterval();
    }
}

START_ROBOT_CLASS(Robot)
