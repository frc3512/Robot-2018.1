// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoAutoLine.hpp"

#include <iostream>

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoAutoLine::AutoAutoLine() { autoTimer.Start(); }

void AutoAutoLine::Reset() { state = State::kInit; }

// Drives forward until passing white line 120 inches away from start
void AutoAutoLine::Run() {
    switch (state) {
        case State::kInit:
            Robot::drivetrain.SetGoal(
                Pose(4.2672 /*168.0*/ - kRobotLength / 2.0, 0.0, 0.0));
            Robot::drivetrain.Enable();
            autoTimer.Reset();

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (Robot::drivetrain.AtGoal()) {
                Robot::drivetrain.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
