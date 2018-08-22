// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoRightDouble.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightDouble::AutoRightDouble() { autoTimer.Start(); }

void AutoRightDouble::Reset() { state = State::kInit; }

void AutoRightDouble::Run() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::drivetrain.SetGoal(
                Pose(236.5 - kRobotLength / 2.0, 0.0, 0.0));
            Robot::drivetrain.Enable();

            Robot::elevator.SetGoal(kScaleHeight);
            Robot::elevator.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();

                state = State::kRightRotate;
            }
            break;
        case State::kRightRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(
                        Pose(20.0 - kRobotLength / 2.0, 0.0, 0.0));
                } else if (platePosition[kFriendlySwitch] == 'R' &&
                           platePosition[kScale] == 'L') {
                    Robot::drivetrain.SetGoal(
                        Pose(20.0 - kRobotLength / 2.0, 0.0, 0.0));

                } else {
                    Robot::drivetrain.SetGoal(
                        Pose(236.5 - kRobotLength / 2.0, 0.0, 0.0));
                }

                state = State::kFirstForward;
            }
            break;
        case State::kFirstForward:
            if (Robot::drivetrain.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 90.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -90.0));
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
                Robot::intake.SetMotors(MotorState::kOuttake);
                autoTimer.Reset();

                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, -180.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(0.0, 0.0, 0.0));
                }

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (Robot::drivetrain.AtGoal()) {
                Robot::intake.SetMotors(MotorState::kIdle);
                Robot::intake.Open();
                Robot::elevator.SetGoal(kFloorHeight);
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetGoal(Pose(60.0, 0.0, 0.0));
                } else {
                    Robot::drivetrain.SetGoal(Pose(10.0, 0.0, 0.0));
                }
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::intake.Close();
                Robot::elevator.SetGoal(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (Robot::elevator.AtReference() &&
                autoTimer.HasPeriodPassed(3.0)) {
                Robot::intake.SetMotors(MotorState::kOuttake);

                Robot::drivetrain.Disable();
                Robot::elevator.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
