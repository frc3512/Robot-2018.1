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

void Robot::AutoCenterPos() {
    static State state = State::kInit;
    static std::string gameData;

    switch (state) {
        case State::kInit:
            gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionReference(50);  // Estimate
            robotDrive.SetAngleReference(0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(90);
                } else {
                    robotDrive.SetAngleReference(-90);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionReference(90);
                } else {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionReference(110);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(-90);
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90);
                }
                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(30);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
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
