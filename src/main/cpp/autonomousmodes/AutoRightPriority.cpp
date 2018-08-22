// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoRightPriority.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightPriority::AutoRightPriority() { autoTimer.Start(); }

void AutoRightPriority::Reset() { state = State::kInit; }

void AutoRightPriority::Run() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'R') {
                Robot::drivetrain.SetGoal(
                    Pose(324.0 - kRobotLength / 2.0, 0.0, 0.0));
                Robot::elevator.SetGoal(kScaleHeight);
                state = State::kInitialForward;
            } else if (platePosition[kFriendlySwitch] == 'R' &&
                       platePosition[kScale] == 'L') {
                Robot::drivetrain.SetGoal(
                    Pose(168.0 - kRobotLength / 2.0, 0.0, 0.0));
                Robot::elevator.SetGoal(kSwitchHeight);

                state = State::kAutoSwitch;
            } else {
                Robot::drivetrain.SetGoal(
                    Pose(168.0 - kRobotLength / 2.0, 0.0, 0.0));

                state = State::kAutoLine;
            }

            Robot::drivetrain.Enable();

            Robot::elevator.Enable();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -88.0));
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(
                    Pose(200.0 + kRobotWidth / 2.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    /*Robot::drivetrain.SetGoal(24.0 + 24.0 -
                                                kRobotLength / 2.0, 0.0, 0.0);*/
                    Robot::drivetrain.SetGoal(Pose(10.0, 0.0, 0.0));
                } else {
                    Robot::drivetrain.SetGoal(
                        Pose(40.0 - kRobotWidth / 2.0 - kRobotLength / 2.0, 0.0,
                             0.0));
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
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoLine:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.Disable();
                Robot::elevator.Disable();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoSwitch:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();

                state = State::kAutoSwitchRotate;
            }
            break;
        case State::kAutoSwitchRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(
                    Pose(65.0 + 12.0 - kRobotLength / 2.0 - kRobotWidth / 2.0,
                         0.0, 0.0));
                autoTimer.Reset();

                state = State::kAutoSwitchForward;
            }
            break;
        case State::kAutoSwitchForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::intake.AutoOuttake();

                autoTimer.Reset();

                Robot::drivetrain.Disable();
                Robot::elevator.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
