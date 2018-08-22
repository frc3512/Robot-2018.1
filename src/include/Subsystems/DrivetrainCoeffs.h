#pragma once

#include <frc/controllers/StateSpaceControllerCoeffs.h>
#include <frc/controllers/StateSpaceLoop.h>
#include <frc/controllers/StateSpaceObserverCoeffs.h>
#include <frc/controllers/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDrivetrainPlantCoeffs();
frc::StateSpaceControllerCoeffs<4, 2, 2> MakeDrivetrainControllerCoeffs();
frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDrivetrainObserverCoeffs();
frc::StateSpaceLoop<4, 2, 2> MakeDrivetrainLoop();
