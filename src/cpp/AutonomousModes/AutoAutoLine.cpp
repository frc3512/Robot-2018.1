// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoAutoLine.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoAutoLine::AutoAutoLine() { autoTimer.Start(); }

void AutoAutoLine::Reset() { state = State::kInit; }

// Drives forward until passing white line 120 inches away from start
void AutoAutoLine::HandleEvent(Event event) {
    switch (state) {
        case State::kInit:
            Robot::robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
            Robot::robotDrive.SetAngleGoal(0.0);
            Robot::robotDrive.StartClosedLoop();
            autoTimer.Reset();

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::robotDrive.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
