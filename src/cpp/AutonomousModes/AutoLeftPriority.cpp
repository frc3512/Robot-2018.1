// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoLeftPriority.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoLeftPriority::AutoLeftPriority() { autoTimer.Start(); }

void AutoLeftPriority::Reset() { state = State::kInit; }

void AutoLeftPriority::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'L') {
                Robot::robotDrive.SetPositionGoal(324.0 - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kScaleHeight);

                state = State::kInitialForward;
            } else if (platePosition[kFriendlySwitch] == 'L' &&
                       platePosition[kScale] == 'R') {
                Robot::robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kAutoSwitch;
            } else {
                Robot::robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);

                state = State::kAutoLine;
            }

            Robot::robotDrive.SetAngleGoal(0.0);
            Robot::robotDrive.StartClosedLoop();

            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kRightRotate;
                }
            }
            break;
        case State::kRightRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(200.0 + kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    /*
                      Robot::robotDrive.SetPositionGoal(24.0 + 6.0 - 6.0 + 24.0
                      - kRobotLength / 2.0);*/
                    Robot::intake.AutoOuttake();
                    state = State::kIdle;
                } else {
                    Robot::robotDrive.SetPositionGoal(40.0 - kRobotWidth / 2.0 -
                                                      kRobotLength / 2.0);
                    state = State::kFinalForward;
                }
                autoTimer.Reset();
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoLine:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoSwitch:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kAutoSwitchRotate;
            }
            break;
        case State::kAutoSwitchRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(65.0 - kRobotLength / 2.0 -
                                                  kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kAutoSwitchForward;
            }
            break;
        case State::kAutoSwitchForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.AutoOuttake();

                autoTimer.Reset();

                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
