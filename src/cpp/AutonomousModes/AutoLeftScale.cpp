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

void Robot::AutoLeftScaleInit() {}

void Robot::AutoLeftScalePeriodic() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'L') {
                robotDrive.SetPositionGoal(
                    260.0 -
                    kRobotLength /
                        2.0);  // Back bumper to middle of robot (ESTIMATE)
            } else {
                robotDrive.SetPositionGoal(228.0 - kRobotLength / 2.0);
            }
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kScaleHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.SetAngleGoal(90.0);
                if (platePosition[kScale] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(137.0);  // Estimate
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'L') {
                    robotDrive.SetPositionGoal(20.0);  // ESTIMATE
                } else {
                    robotDrive.SetPositionGoal(50.0);  // ESTIMATE
                }                                      // Estimate
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
