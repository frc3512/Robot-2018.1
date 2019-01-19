// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "autonomousmodes/AutoLeftDouble.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftDouble::AutoLeftDouble() { autoTimer.Start(); }

void AutoLeftDouble::Reset() { state = State::kInit; }

void AutoLeftDouble::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetGoal(Pose(2.0, 2.0, 1.57));
            Robot::robotDrive.Enable();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtGoal()) {
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
