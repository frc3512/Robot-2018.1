// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controllers/StateSpaceControllerCoeffs.h>
#include <frc/controllers/StateSpaceLoop.h>
#include <frc/controllers/StateSpaceObserverCoeffs.h>
#include <frc/controllers/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<1, 1, 1> MakeFlywheelPlantCoeffs();
frc::StateSpaceControllerCoeffs<1, 1, 1> MakeFlywheelControllerCoeffs();
frc::StateSpaceObserverCoeffs<1, 1, 1> MakeFlywheelObserverCoeffs();
frc::StateSpaceLoop<1, 1, 1> MakeFlywheelLoop();
