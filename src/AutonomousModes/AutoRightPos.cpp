// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

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

void Robot::AutoRightPos() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            if (gameData[0] == 'L') {
                robotDrive.SetPositionReference(150);
            } else {
                robotDrive.SetPositionReference(200);
            }

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference()) {
                if (gameData[0] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(-90);
                    state = State::kLeftForward;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(200);  // Estimate
                state = State::kLeftForward;
            }
        case State::kLeftForward:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(-90);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(15);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference()) {
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
