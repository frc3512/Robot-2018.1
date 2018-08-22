// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoLeftSwitch.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoLeftSwitch::AutoLeftSwitch() { autoTimer.Start(); }

void AutoLeftSwitch::Reset() { state = State::kInit; }

void AutoLeftSwitch::HandleEvent(Event event) {
    static std::string platePosition;
    static frc::TrapezoidProfile positionProfile;
    static frc::TrapezoidProfile angleProfile;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'L') {
                positionProfile.SetGoal(168.0 - kRobotLength / 2.0);
            } else {
                positionProfile.SetGoal(252.0 - kRobotLength / 2.0);
            }
            anglePosition.SetGoal(0.0);
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                anglePosition.SetGoal(90.0);
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (anglePosition.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                positionProfile.SetGoal(190.0 - 50.0);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                anglePosition.SetGoal(90.0 - 50.0 + 10.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (anglePosition.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'L') {
                    positionProfile.SetGoal(
                        65.0 - kRobotLength / 2.0 - kRobotWidth / 2.0);
                } else {
                    positionProfile.SetGoal(
                        36.0 + 24.0 - kRobotLength / 2.0);  // 28.0
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 0.1) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.Disable();
                Robot::elevator.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
