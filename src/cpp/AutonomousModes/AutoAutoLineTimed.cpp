// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

// Incase nothing is working and we want to attempt a ranking point by getting
// past the autoline
void Robot::AutoAutoLineTimed() {
    robotDrive.Drive(0.5, 0.0, false);
    if (autoTimer.HasPeriodPassed(7.0)) {  // Estimate
        robotDrive.Drive(0.0, 0.0, false);
    }
}
