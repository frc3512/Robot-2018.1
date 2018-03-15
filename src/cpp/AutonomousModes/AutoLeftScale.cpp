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
    kAutoLine,
    kAutoSwitchRotate,
    kAutoSwitchForward,
    kAutoSwitch,
    kIdle
};

static State state;

void Robot::AutoLeftScaleInit() { state = State::kInit; }

void Robot::AutoLeftScalePeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'L') {
                robotDrive.SetPositionGoal(328.0 - kRobotLength / 2.0);
                elevator.SetHeightReference(kScaleHeight);

                state = State::kInitialForward;
            } else if (platePosition[kFriendlySwitch] == 'L' &&
                       platePosition[kScale] == 'R') {
                robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
                elevator.SetHeightReference(kSwitchHeight);

                state = State::kAutoSwitch;
            } else {
                robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);

                state = State::kAutoLine;
            }

            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.StartClosedLoop();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
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
                robotDrive.SetPositionGoal(200.0 + kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    robotDrive.SetPositionGoal(24.0 + 6.0 - kRobotLength / 2.0);
                } else {
                    robotDrive.SetPositionGoal(40.0 - kRobotWidth / 2.0 -
                                               kRobotLength / 2.0);
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                intake.AutoOuttake();
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoLine:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoSwitch:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kAutoSwitchRotate;
            }
            break;
        case State::kAutoSwitchRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(65.0 - kRobotLength / 2.0 -
                                           kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kAutoSwitchForward;
            }
            break;
        case State::kAutoSwitchForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                intake.AutoOuttake();

                autoTimer.Reset();

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
