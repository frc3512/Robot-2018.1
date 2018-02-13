// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <iostream>

#include "Robot.hpp"

enum class State { kInit, kMoveForward, kIdle };

void Robot::AutoAutoLineInit() {}

// Drives forward until passing white line 120 inches away from start
void Robot::AutoAutoLinePeriodic() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.SetPositionGoal(kRobotLength + 120.0);  // Estimate
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            state = State::kMoveForward;
            std::cout << "Move Forward" << std::endl;
            break;
        case State::kMoveForward:
            if (robotDrive.AtPositionGoal()) {
                robotDrive.StopClosedLoop();
                std::cout << "Idle" << std::endl;
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
