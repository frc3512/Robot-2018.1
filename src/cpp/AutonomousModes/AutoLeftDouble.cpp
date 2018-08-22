// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoLeftDouble.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

#include "WPILib/CtrlSys/TrapezoidProfile.h"

AutoLeftDouble::AutoLeftDouble() { autoTimer.Start(); }

void AutoLeftDouble::Reset() { state = State::kInit; }

void AutoLeftDouble::HandleEvent(Event event) {
    static std::string platePosition;
    static frc::TrapezoidProfile positionProfile;
    static frc::TrapezoidProfile angleProfile;
   
    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            positionProfile.SetGoal(236.5 - kRobotLength / 2.0);
            angleProfile.SetGoal(236.5 - kRobotLength / 2.0);
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kScaleHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                angleProfile.SetGoal(90.0);
                autoTimer.Reset();

                state = State::kLeftRotate;
            }
            break;
        case State::kLeftRotate:
            if (Robot::robotDrive.AtReference() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    positionProfile.SetGoal(20.0);
                } else {
                    positionProfile.SetGoal(137.0);  // Estimate
                }

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                anglePosition.SetGoal(-90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                positionProfile.SetGoal(50.0);  // ESTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (positionProfile.AtPositionGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::intake.Open();
                Robot::robotDrive.ResetGyro();
                angleProfile.SetGoal(180.0);
                autoTimer.Reset();

                state = State::kDoubleRotate;
            }
            break;

        case State::kDoubleRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::elevator.SetHeightReference(kFloorHeight);
                Robot::robotDrive.ResetEncoders();
                positionProfile.SetGoal(60.0);
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::intake.Close();
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (Robot::elevator.HeightAtReference() ||
                autoTimer.HasPeriodPassed(3.0)) {
                Robot::intake.SetMotors(MotorState::kOuttake);

                Robot::drivetrain.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
