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

void Robot::AutoLeftSwitch() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'L') {
                robotDrive.SetPositionReference(
                    168.0 -
                    kRobotLength / 2.0);  // Back bumper to middle of robot
            } else {
                robotDrive.SetPositionReference(228.0 - kRobotLength / 2.0);
            }
            robotDrive.SetAngleReference(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetAngleReference(90.0);
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90.0);
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(137.0);  // Estimate
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(90.0);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(20.0);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
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
