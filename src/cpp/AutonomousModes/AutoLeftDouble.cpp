// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoLeftDouble.hpp"

#include <DriverStation.h>
#include <WPILib/CtrlSys/TrapezoidProfile.h>

#include "Robot.hpp"

AutoLeftDouble::AutoLeftDouble() { autoTimer.Start(); }

void AutoLeftDouble::Reset() { state = State::kInit; }

void AutoLeftDouble::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetGoal(236.5 - kRobotLength / 2.0, 0.0, 0.0);
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kScaleHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() > positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.SetGoal(0.0, 0.0, 90.0);
                autoTimer.Reset();

                state = State::kLeftRotate;
            }
            break;
        case State::kLeftRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() > angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    Robot::robotDrive.SetGoal(20.0, 0.0, 0.0);
                } else {
                    Robot::robotDrive.SetGoal(137.0, 0.0, 0.0);  // Estimate
                }

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() > positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetGoal(0.0, 0.0, -90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() > angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(50.0, 0.0, 0.0);  // ESTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() > positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::intake.Open();
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetGoal(0.0, 0.0, 180.0);
                autoTimer.Reset();

                state = State::kDoubleRotate;
            }
            break;

        case State::kDoubleRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() > angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::elevator.SetHeightReference(kFloorHeight);
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(60.0, 0.0, 0.0);
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() > positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::intake.Close();
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (Robot::elevator.HeightAtReference() ||
                autoTimer.HasPeriodPassed(3.0)) {
                Robot::intake.SetMotors(MotorState::kOuttake);

                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
