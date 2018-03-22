// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoRightSwitch.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoRightSwitch::AutoRightSwitch() { autoTimer.Start(); }

void AutoRightSwitch::Reset() { state = State::kInit; }

void AutoRightSwitch::HandleEvent(Event event) {
    static std::string platePosition;
    bool makeTransition = false;
    State nextState;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (event.type == EventType::kEntry) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetPositionGoal(168.0 -
                                                      kRobotLength / 2.0);
                } else {
                    Robot::robotDrive.SetPositionGoal(252.0 -
                                                      kRobotLength / 2.0);
                }
                Robot::robotDrive.SetAngleGoal(0.0);
                Robot::robotDrive.StartClosedLoop();

                Robot::elevator.SetHeightReference(kSwitchHeight);
                Robot::elevator.StartClosedLoop();

                autoTimer.Reset();
            } else if (Robot::robotDrive.AtPositionGoal() ||
                       autoTimer.Get() >
                           Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    nextState = State::kFinalRotate;
                } else {
                    nextState = State::kRightRotate;
                }
                makeTransition = true;
            }
            break;
        case State::kRightRotate:
            if (event.type == EventType::kEntry) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();
            } else if (Robot::robotDrive.AtAngleGoal() ||
                       autoTimer.Get() >
                           Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                nextState = State::kRightForward;
                makeTransition = true;
            }
            break;
        case State::kRightForward:
            if (event.type == EventType::kEntry) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(190.0);
                autoTimer.Reset();
            } else if (Robot::robotDrive.AtPositionGoal() ||
                       autoTimer.Get() >
                           Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                nextState = State::kFinalRotate;
                makeTransition = true;
            }
            break;
        case State::kFinalRotate:
            if (event.type == EventType::kEntry) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();
            } else if (Robot::robotDrive.AtAngleGoal() ||
                       autoTimer.Get() >
                           Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                nextState = State::kFinalForward;
                makeTransition = true;
            }
            break;
        case State::kFinalForward:
            if (event.type == EventType::kEntry) {
                Robot::robotDrive.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetPositionGoal(
                        65.0 - kRobotLength / 2.0 - kRobotWidth / 2.0);  // 55
                } else {
                    Robot::robotDrive.SetPositionGoal(36.0 -
                                                      kRobotLength / 2.0);
                }
                autoTimer.Reset();

            } else if (Robot::robotDrive.AtPositionGoal() ||
                       autoTimer.Get() >
                           Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                nextState = State::kIdle;
                makeTransition = true;
            }
            break;
        case State::kIdle:
            if (event.type == EventType::kEntry) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();
            }
            break;
    }

    if (makeTransition) {
        PostEvent(EventType::kExit);
        state = nextState;
        PostEvent(EventType::kEntry);
    }
}
