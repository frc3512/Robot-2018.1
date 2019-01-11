// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoRightPriority.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightPriority::AutoRightPriority() { autoTimer.Start(); }

void AutoRightPriority::Reset() { state = State::kInit; }

void AutoRightPriority::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'R') {
                Robot::robotDrive.SetGoal(
                    Pose(324.0 - kRobotLength / 2.0, 0.0, 0.0));
                Robot::elevator.SetHeightReference(kScaleHeight);

                state = State::kInitialForward;
            } else if (platePosition[kFriendlySwitch] == 'R' &&
                       platePosition[kScale] == 'L') {
                Robot::robotDrive.SetGoal(
                    Pose(168.0 - kRobotLength / 2.0, 0.0, 0.0));
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kAutoSwitch;
            } else {
                Robot::robotDrive.SetGoal(
                    Pose(168.0 - kRobotLength / 2.0, 0.0, 0.0));

                state = State::kAutoLine;
            }

            Robot::robotDrive.Enable();

            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -88.0));
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(
                    Pose(200.0 + kRobotWidth / 2.0, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    /*Robot::robotDrive.SetGoal(24.0 + 24.0 -
                                                kRobotLength / 2.0, 0.0, 0.0);*/
                    Robot::robotDrive.SetGoal(Pose(10.0, 0.0, 0.0));
                } else {
                    Robot::robotDrive.SetGoal(
                        Pose(40.0 - kRobotWidth / 2.0 - kRobotLength / 2.0, 0.0,
                             0.0));
                }
                autoTimer.Reset();
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoLine:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoSwitch:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.SetGoal(Pose(0.0, 0.0, -90.0));
                autoTimer.Reset();

                state = State::kAutoSwitchRotate;
            }
            break;
        case State::kAutoSwitchRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(
                    Pose(65.0 + 12.0 - kRobotLength / 2.0 - kRobotWidth / 2.0,
                         0.0, 0.0));
                autoTimer.Reset();

                state = State::kAutoSwitchForward;
            }
            break;
        case State::kAutoSwitchForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::intake.AutoOuttake();

                autoTimer.Reset();

                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
