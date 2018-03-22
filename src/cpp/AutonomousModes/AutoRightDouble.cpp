// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoRightDouble.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoRightDouble::AutoRightDouble() { autoTimer.Start(); }

void AutoRightDouble::Reset() { state = State::kInit; }

void AutoRightDouble::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetPositionGoal(236.5 - kRobotLength / 2.0);
            Robot::robotDrive.SetAngleGoal(0.0);
            Robot::robotDrive.StartClosedLoop();

            Robot::elevator.SetHeightReference(kScaleHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kRightRotate;
            }
            break;
        case State::kRightRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetPositionGoal(20.0 -
                                                      kRobotLength / 2.0);
                } else if (platePosition[kFriendlySwitch] == 'R' &&
                           platePosition[kScale] == 'L') {
                    Robot::robotDrive.SetPositionGoal(20.0 -
                                                      kRobotLength / 2.0);

                } else {
                    Robot::robotDrive.SetPositionGoal(236.5 -
                                                      kRobotLength / 2.0);
                }

                state = State::kFirstForward;
            }
            break;
        case State::kFirstForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::robotDrive.SetAngleGoal(90);
                } else {
                    Robot::robotDrive.SetAngleGoal(-90);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(40.0 - kRobotLength /
                                                             2.0);  // ESTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.SetMotors(MotorState::kOuttake);
                Robot::robotDrive.ResetGyro();
                autoTimer.Reset();

                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetAngleGoal(-180.0);
                } else {
                    Robot::robotDrive.SetAngleGoal(0.0);
                }

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::intake.SetMotors(MotorState::kIdle);
                Robot::intake.Open();
                Robot::elevator.SetHeightReference(kFloorHeight);
                Robot::robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetPositionGoal(60.0);
                } else {
                    Robot::robotDrive.SetPositionGoal(10.0);
                }
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.Close();
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (Robot::elevator.HeightAtReference() &&
                autoTimer.HasPeriodPassed(3.0)) {
                Robot::intake.SetMotors(MotorState::kOuttake);

                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
