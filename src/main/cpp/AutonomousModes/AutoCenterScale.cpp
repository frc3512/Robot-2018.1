// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoCenterScale.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoCenterScale::AutoCenterScale() { autoTimer.Reset(); }

void AutoCenterScale::Reset() { state = State::kInit; }

void AutoCenterScale::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetGoal(
                Pose(67.0 - kRobotLength / 2.0, 0.0, 0.0));
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, 90.0));
                } else {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -90.0));
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetGoal(
                        Pose(132.0 - kExchangeOffset - kRobotLength / 2.0, 0.0,
                             0.0));
                } else {
                    Robot::robotDrive.SetGoal(
                        Pose(132.0 + kExchangeOffset - kRobotLength / 2.0, 0.0,
                             0.0));
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::robotDrive.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -90.0));
                } else {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, 90.0));
                }

                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(Pose(260.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (Robot::robotDrive.AtGoal()) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -90.0));
                } else {
                    Robot::robotDrive.SetGoal(Pose(0.0, 0.0, 90.0));
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(
                    Pose(40.0 - kRobotLength / 2.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::intake.Open();
                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
