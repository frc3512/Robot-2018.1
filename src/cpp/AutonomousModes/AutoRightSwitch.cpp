// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kRightRotate,
    kRightForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoRightSwitchInit() {}

void Robot::AutoRightSwitchPeriodic() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'R') {
                robotDrive.SetPositionReference(
                    168.0 -
                    kRobotLength / 2.0);  // Back bumper to middle of robot
            } else {
                robotDrive.SetPositionReference(228.0 - kRobotLength / 2.0);
            }
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90.0);
                    state = State::kRightForward;
                }
            }
            break;
        case State::kRightRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(137.0);  // Estimate
                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();  // For Simplicity

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
                robotDrive.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
