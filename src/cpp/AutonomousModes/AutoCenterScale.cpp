// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoCenterScale.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoCenterScale::AutoCenterScale() { autoTimer.Reset(); }

void AutoCenterScale::Reset() { state = State::kInit; }

void AutoCenterScale::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetPositionGoal(67.0 -
                                              kRobotLength / 2.0);  // Estimate
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
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetAngleGoal(90.0);
                } else {
                    Robot::robotDrive.SetAngleGoal(-90.0);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetPositionGoal(132.0 - kExchangeOffset -
                                                      kRobotLength / 2.0);
                } else {
                    Robot::robotDrive.SetPositionGoal(132.0 + kExchangeOffset -
                                                      kRobotLength /
                                                          2.0);  // ESTIMATE
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetAngleGoal(-90.0);
                } else {
                    Robot::robotDrive.SetAngleGoal(90.0);
                }

                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(260.0);
                autoTimer.Reset();

                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetAngleGoal(-90.0);
                } else {
                    Robot::robotDrive.SetAngleGoal(90.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(40.0 - kRobotLength / 2.0);
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.Open();
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
