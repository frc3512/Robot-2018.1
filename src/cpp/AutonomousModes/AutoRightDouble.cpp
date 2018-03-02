// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kRightRotate,
    kFirstForward,
    kFinalRotate,
    kFinalForward,
    kDoubleRotate,
    kDoubleForward,
    kSpit,
    kIdle
};

static State state;

void Robot::AutoRightDoubleInit() { state = State::kInit; }

void Robot::AutoRightDoublePeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionGoal(236.5 - kRobotLength / 2.0);
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kScaleHeight);
            elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kRightRotate;
            }
            break;
        case State::kRightRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    robotDrive.SetPositionGoal(20.0 - kRobotLength / 2.0);
                } else if (platePosition[kFriendlySwitch] == 'R' &&
                           platePosition[kScale] == 'L') {
                    robotDrive.SetPositionGoal(20.0 - kRobotLength / 2.0);

                } else {
                    robotDrive.SetPositionGoal(236.5 - kRobotLength / 2.0);
                }

                state = State::kFirstForward;
            }
            break;
        case State::kFirstForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    robotDrive.SetAngleGoal(90);
                } else {
                    robotDrive.SetAngleGoal(-90);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(40.0 -
                                           kRobotLength / 2.0);  // ESTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                intake.SetMotors(MotorState::kOuttake);
                robotDrive.ResetGyro();
                autoTimer.Reset();

                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetAngleGoal(-180.0);
                } else {
                    robotDrive.SetAngleGoal(0.0);
                }

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                intake.SetMotors(MotorState::kIdle);
                intake.Open();
                elevator.SetHeightReference(kFloorHeight);
                robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetPositionGoal(60.0);
                } else {
                    robotDrive.SetPositionGoal(10.0);
                }
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                intake.Close();
                elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (elevator.HeightAtReference() &&
                autoTimer.HasPeriodPassed(3.0)) {
                intake.SetMotors(MotorState::kOuttake);

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
