// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controllers/StateSpaceControllerCoeffs.h>
#include <frc/controllers/StateSpaceLoop.h>
#include <frc/controllers/StateSpaceObserverCoeffs.h>
#include <frc/controllers/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeElevatorPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 1, 1> MakeElevatorControllerCoeffs();
frc::StateSpaceObserverCoeffs<2, 1, 1> MakeElevatorObserverCoeffs();
frc::StateSpaceLoop<2, 1, 1> MakeElevatorLoop();
