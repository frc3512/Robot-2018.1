// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoCenterSwitchInit() {}

void Robot::AutoCenterSwitchPeriodic() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionReference(50.0);  // Estimate
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
                } else {
                    robotDrive.SetAngleReference(-90.0);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionReference(90.0);
                } else {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionReference(110.0);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(-90.0);
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90.0);
                }
                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(30.0);  // Estimate
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
