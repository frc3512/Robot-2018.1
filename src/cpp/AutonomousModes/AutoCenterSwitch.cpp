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

            robotDrive.SetPositionGoal(50.0);  // Estimate
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetAngleGoal(90.0);
                } else {
                    robotDrive.SetAngleGoal(-90.0);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionGoal(90.0);
                } else {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionGoal(110.0);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(-90.0);
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(90.0);
                }
                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(30.0);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1.0)) {
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
