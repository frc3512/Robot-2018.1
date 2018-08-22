// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoCenterScale.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoCenterScale::AutoCenterScale() { autoTimer.Reset(); }

void AutoCenterScale::Reset() { state = State::kInit; }

void AutoCenterScale::Run() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::drivetrain.SetGoal(
                Pose(67.0 - kRobotLength / 2.0, 0.0, 0.0));
            Robot::drivetrain.Enable();

            Robot::elevator.SetGoal(kSwitchHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 90.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(
                        Pose(132.0 - kExchangeOffset - kRobotLength / 2.0, 0.0,
                             0.0));
                } else {
                    Robot::drivetrain.SetGoal(
                        Pose(132.0 + kExchangeOffset - kRobotLength / 2.0, 0.0,
                             0.0));
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::drivetrain.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 90.0));
                }

                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(Pose(260.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (Robot::drivetrain.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 90.0));
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(
                    Pose(40.0 - kRobotLength / 2.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::intake.Open();
                Robot::drivetrain.Disable();
                Robot::elevator.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
