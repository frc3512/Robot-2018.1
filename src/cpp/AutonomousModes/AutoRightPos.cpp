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

void Robot::AutoRightPos() {
    static State state = State::kInit;
    static std::string gameData;

    switch (state) {
        case State::kInit:
            gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (gameData[0] == 'R') {
                robotDrive.SetPositionReference(
                    168 - kRobotLength / 2);  // Back bumper to middle of robot
            } else {
                robotDrive.SetPositionReference(228 - kRobotLength / 2);
            }
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90);
                    state = State::kRightForward;
                }
            }
            break;
        case State::kRightRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(137);  // Estimate
                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();  // For Simplicity

                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(90);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(20);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
