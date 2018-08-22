// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoAutoLine.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoAutoLine::AutoAutoLine() { autoTimer.Start(); }

void AutoAutoLine::Reset() { state = State::kInit; }

// Drives forward until passing white line 120 inches away from start
void AutoAutoLine::HandleEvent(Event event) {
    static frc::TrapezoidProfile positionProfile(kRobotMaxV, kRobotTimeToMaxV);
    static frc::TrapezoidProfile angleProfile(kRobotMaxRotateRate,
                                              kRobotTimeToMaxRotateRate);

    switch (state) {
        case State::kInit:
            positionProfile.SetGoal(168.0 - kRobotLength / 2.0);
            angleProfile.SetGoal(0.0);
            Robot::robotDrive.Enable();
            autoTimer.Reset();

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (positionProfile.AtGoal() ||
                autoTimer.Get() > positionProfile.ProfileTimeTotal() + 1.0) {
                Robot::robotDrive.Disable();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
