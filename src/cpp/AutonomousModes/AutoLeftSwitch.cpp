// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoLeftSwitch.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoLeftSwitch::AutoLeftSwitch() { autoTimer.Start(); }

void AutoLeftSwitch::Reset() { state = State::kInit; }

void AutoLeftSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'L') {
                Robot::robotDrive.SetGoal(
                    Pose(4.2672 - kRobotLength / 2.0, 0.0, 0.0));
            } else {
                Robot::robotDrive.SetGoal(
                    Pose(6.4008 - kRobotLength / 2.0, 0.0, 0.0));
            }
            Robot::robotDrive.Enable();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.SetGoal(Pose(0.0, 0.0, 1.5708));
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetGoal(Pose(3.556, 0.0, 0.0));
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.SetGoal(Pose(0.0, 0.0, 0.872665));
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtGoal()) {
                Robot::robotDrive.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'L') {
                    Robot::robotDrive.SetGoal(
                        Pose(1.651 - kRobotLength / 2.0 - kRobotWidth / 2.0,
                             0.0, 0.0));
                } else {
                    Robot::robotDrive.SetGoal(
                        Pose(1.524 - kRobotLength / 2.0, 0.0, 0.0));
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtGoal()) {
                Robot::intake.AutoOuttake();
                Robot::robotDrive.Disable();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
