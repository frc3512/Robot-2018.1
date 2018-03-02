// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kLeftRotate,
    kLeftForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoRightScaleInit() { state = State::kInit; }

void Robot::AutoRightScalePeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'R') {
                robotDrive.SetPositionGoal(324.0 - kRobotLength / 2.0);
            } else {
                robotDrive.SetPositionGoal(236.5 - kRobotLength / 2.0);
            }
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
                if (platePosition[kScale] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(199.0);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetPositionGoal(24.0 - kRobotLength / 2.0);
                } else {
                    robotDrive.SetPositionGoal(56.0 - kRobotLength / 2.0);
                }

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                intake.Open();

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
