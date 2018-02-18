// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});
    dsDisplay.AddAutoMethod("Autoline Timed",
                            std::bind(&Robot::AutoAutoLineTimed, this));
    dsDisplay.AddAutoMethod("Autoline", std::bind(&Robot::AutoAutoLine, this));
    dsDisplay.AddAutoMethod("Left Position Switch",
                            std::bind(&Robot::AutoLeftSwitch, this));
    dsDisplay.AddAutoMethod("Center Position Switch",
                            std::bind(&Robot::AutoCenterSwitch, this));
    dsDisplay.AddAutoMethod("Right Position Switch",
                            std::bind(&Robot::AutoRightSwitch, this));
    dsDisplay.AddAutoMethod("Left Position Scale",
                            std::bind(&Robot::AutoLeftScale, this));
    dsDisplay.AddAutoMethod("Center Position Scale",
                            std::bind(&Robot::AutoCenterScale, this));
    dsDisplay.AddAutoMethod("Right Position Scale",
                            std::bind(&Robot::AutoRightScale, this));
    server.SetSource(camera1);

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);

    // camera2.SetResolution(640, 480);
    // camera2.SetFPS(30);
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    robotDrive.ResetGyro();
    robotDrive.ResetEncoders();
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
    elevatorMode = ElevatorMode::kPosition;
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
    robotDrive.StopClosedLoop();
    elevator.StartClosedLoop();
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    DS_PrintOut();

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

void Robot::AutonomousPeriodic() { dsDisplay.ExecAutonomous(); }

void Robot::TeleopPeriodic() {
    // Drive Stick Controls
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }

    // Intake Controls
    if (appendageStick.GetRawButtonPressed(2)) {
        climber.Shift();
    }

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

    switch (elevatorMode) {
        case ElevatorMode::kPosition:  // TODO: change the Height
                                       // References back to constants when
                                       // we know the correct heights
            if (!elevator.GetForwardHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButton(7)) {
                elevator.SetHeightReference(kFloorHeight);
            }

            if (appendageStick.GetRawButton(8)) {
                elevator.SetHeightReference(/*kSwitchHeight*/ -8.0);
            }
            if (appendageStick.GetRawButton(9)) {
                elevator.SetHeightReference(/*kScaleHeight*/ -20.0);
            }
            if (appendageStick.GetRawButton(10)) {
                elevator.SetHeightReference(/*kClimbHeight*/ -80.0);
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StopClosedLoop();
                elevatorMode = ElevatorMode::kVelocity;
            }
            break;
        case ElevatorMode::kVelocity:
            elevator.SetVelocity(appendageStick.GetY());
            if (!elevator.GetForwardHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StartClosedLoop();
                elevatorMode = ElevatorMode::kPosition;
            }
    }

    /*if (appendageStick.GetRawButtonPressed(11)) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }*/
}

void Robot::DS_PrintOut() {
    // robotDrive.Debug();
    /*if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData(robotDrive.GetAngleReference(),
                              "Angle Reference");
        liveGrapher.GraphData(robotDrive.GetAngle(), "Angle");
        liveGrapher.GraphData(elevator.GetHeight(), "Elevator Height");
        liveGrapher.GraphData(elevator.GetHeightReference(), "Elevator
    Reference");

        liveGrapher.GraphData(robotDrive.GetPosReference(), "Position
    Reference"); liveGrapher.GraphData(robotDrive.GetPosition(), "Position");
        liveGrapher.ResetInterval();
    }*/
}

START_ROBOT_CLASS(Robot)
