// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kSecondRotate,
    kThirdForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoCenterScaleInit() { state = State::kInit; }

void Robot::AutoCenterScalePeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionGoal(67.0 - kRobotLength / 2.0);  // Estimate
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleGoal(90.0);
                } else {
                    robotDrive.SetAngleGoal(-90.0);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetPositionGoal(132.0 - kExchangeOffset -
                                               kRobotLength / 2.0);
                } else {
                    robotDrive.SetPositionGoal(132.0 + kExchangeOffset -
                                               kRobotLength / 2.0);  // ESTIMATE
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleGoal(-90.0);
                } else {
                    robotDrive.SetAngleGoal(90.0);
                }

                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(260.0);
                autoTimer.Reset();

                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 1.0) {
                robotDrive.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleGoal(-90.0);
                } else {
                    robotDrive.SetAngleGoal(90.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 1.0) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(40.0 - kRobotLength / 2.0);
                autoTimer.Reset();

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
