// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { kInit, kMoveForward, kIdle };

// Drives forward until passing white line 120 inches away from start
void Robot::AutoAutoLine() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            robotDrive.SetPositionReference(kRobotLength + 120);  // Estimate
            robotDrive.SetAngleReference(0);

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
