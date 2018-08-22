// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoCenterSwitch.hpp"

#include <cmath>

#include <DriverStation.h>

#include "Robot.hpp"

AutoCenterSwitch::AutoCenterSwitch() { autoTimer.Start(); }

void AutoCenterSwitch::Reset() { state = State::kInit; }

void AutoCenterSwitch::HandleEvent(Event event) {
    static std::string platePosition;
    static frc::TrapezoidProfile positionProfile;
    static frc::TrapezoidProfile angleProfile;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            positionProfile.SetGoal(kRobotWidth / 2.0);
            angleProfile.SetGoal(0.0);
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 1.0) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    angleProfile.SetGoal(
                        rad2deg(std::atan2(58.44 + kExchangeOffset,
                                           140 - kRobotWidth)) -
                        10);
                } else {
                    angleProfile.SetGoal(
                        rad2deg(std::atan2(-76.44 - kExchangeOffset,
                                           140 - kRobotWidth)) +
                        10);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    positionProfile.SetGoal(
                        std::sqrt(std::pow(58.44 - kExchangeOffset, 2) +
                                  std::pow(140 - kRobotWidth, 2)) +
                        6);
                } else {
                    positionProfile.SetGoal(
                        std::sqrt(std::pow(76.44 + kExchangeOffset, 2) +
                                  std::pow(140 - kRobotWidth, 2)) -
                        9);
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 0.1) {
                autoTimer.Reset();
                /*
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.ResetGyro();
                    angleProfile.SetGoal(rad2deg(std::atan2(
                        -140 + kRobotWidth, 58.44 - kExchangeOffset)));
                } else {
                    Robot::robotDrive.ResetGyro();
                    angleProfile.SetGoal(-rad2deg(std::atan2(
                        -76.44 - kExchangeOffset, 140 - kRobotWidth)) - 10);
                }*/
                Robot::intake.AutoOuttake();

                // state = State::kFinalRotate;
                state = State::kIdle;
            }
            break;
        case State::kFinalRotate:
            if (angleProfile.AtGoal() ||
                autoTimer.Get() >
                    angleProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                positionProfile.SetGoal(kRobotWidth / 2 - 3);
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() >
                    positionProfile.ProfileTimeTotal() + 0.1) {
                Robot::intake.SetMotors(MotorState::kOuttake);
                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            Robot::robotDrive.Disable();
            break;
    }
}
