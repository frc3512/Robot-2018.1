// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoRightSwitch.hpp"

#include <iostream>

#include <DriverStation.h>

#include "Robot.hpp"

AutoRightSwitch::AutoRightSwitch() { autoTimer.Start(); }

void AutoRightSwitch::Reset() { state = State::kInit; }

void AutoRightSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'R') {
                Robot::robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
            } else if (platePosition[kFriendlySwitch] == 'L') {
                Robot::robotDrive.SetPositionGoal(252.0 - kRobotLength / 2.0 -
                                                  12.0);
            } else {
                std::cout << "THIS IS TERRIBLE" << std::endl;
            }
            Robot::robotDrive.SetAngleGoal(0.0);
            Robot::robotDrive.StartClosedLoop();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                Robot::robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
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
                Robot::robotDrive.SetPositionGoal(190.0);
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
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetPositionGoal(
                        65.0 - kRobotLength / 2.0 - kRobotWidth / 2.0);
                } else {
                    Robot::robotDrive.SetPositionGoal(36.0 - kRobotLength /
                                                                 2.0);  // 28.0
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
