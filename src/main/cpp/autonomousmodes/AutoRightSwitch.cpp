// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoRightSwitch.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightSwitch::AutoRightSwitch() { autoTimer.Start(); }

void AutoRightSwitch::Reset() { state = State::kInit; }

void AutoRightSwitch::Run() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'R') {
                Robot::drivetrain.SetGoal(
                    Pose(168.0 - kRobotLength / 2.0, 0.0, 0.0));
            } else {
                Robot::drivetrain.SetGoal(
                    Pose(252.0 - 17.0 - kRobotLength / 2.0, 0.0, 0.0));
            }
            Robot::drivetrain.Enable();

            Robot::elevator.SetGoal(kSwitchHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kRightRotate;
                }
            }
            break;
        case State::kRightRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(Pose(190.0 - 38.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0 + 35.0));
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(
                        65.0 + 12.0 - kRobotLength / 2.0 - kRobotWidth / 2.0,
                        0.0, 0.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(
                        36.0 + 24.0 - kRobotLength / 2.0, 0.0, 0.0));  // 28.0
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::intake.AutoOuttake();
                Robot::drivetrain.Disable();
                Robot::elevator.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
