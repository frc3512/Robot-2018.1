// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoCenterSwitch.hpp"

#include <cmath>

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoCenterSwitch::AutoCenterSwitch() { autoTimer.Start(); }

void AutoCenterSwitch::Reset() { state = State::kInit; }

void AutoCenterSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetGoal(Pose(kRobotWidth / 2.0, 0.0, 0.0));
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetGoal(
                        Pose(0.0, 0.0,
                             std::atan2(1.484376 + kExchangeOffset,
                                        3.556 - kRobotWidth) -
                                 deg2rad(10)));
                } else {
                    Robot::robotDrive.SetGoal(
                        Pose(0.0, 0.0,
                             std::atan2(-1.941576 - kExchangeOffset,
                                        3.556 - kRobotWidth) +
                                 deg2rad(10)));
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetGoal(
                        Pose(std::sqrt(std::pow(1.48437 - kExchangeOffset, 2) +
                                       std::pow(3.556 - kRobotWidth, 2)) +
                                 0.1524,
                             0.0, 0.0));
                } else {
                    Robot::robotDrive.SetGoal(
                        Pose(std::sqrt(std::pow(1.941576 + kExchangeOffset, 2) +
                                       std::pow(3.556 - kRobotWidth, 2)) -
                                 0.2286,
                             0.0, 0.0));
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::robotDrive.AtGoal()) {
                autoTimer.Reset();
                /*
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetGoal(rad2deg(0.0, 0.0, std::atan2(
                        -140 + kRobotWidth, 58.44 - kExchangeOffset)));
                } else {
                    Robot::robotDrive.SetGoal(0.0, 0.0, -rad2deg(std::atan2(
                        -76.44 - kExchangeOffset, 140 - kRobotWidth)) - 10);
                }*/
                Robot::intake.AutoOuttake();

                // state = State::kFinalRotate;
                state = State::kIdle;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(
                    Pose(kRobotWidth / 2 - 0.0762, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtGoal()) {
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
