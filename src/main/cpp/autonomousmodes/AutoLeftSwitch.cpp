// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoLeftSwitch.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftSwitch::AutoLeftSwitch() { autoTimer.Start(); }

void AutoLeftSwitch::Reset() { state = State::kInit; }

void AutoLeftSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'L') {
                Robot::drivetrain.SetGoal(
                    Pose(4.2672 - kRobotLength / 2.0, 0.0, 0.0));
            } else {
                Robot::drivetrain.SetGoal(
                    Pose(6.4008 - kRobotLength / 2.0, 0.0, 0.0));
            }
            Robot::drivetrain.Enable();

            Robot::elevator.SetGoal(kSwitchHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 1.5708));
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetGoal(Pose(3.556, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 0.872665));
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'L') {
                    Robot::drivetrain.SetGoal(
                        Pose(1.651 - kRobotLength / 2.0 - kRobotWidth / 2.0,
                             0.0, 0.0));
                } else {
                    Robot::drivetrain.SetGoal(
                        Pose(1.524 - kRobotLength / 2.0, 0.0, 0.0));
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
