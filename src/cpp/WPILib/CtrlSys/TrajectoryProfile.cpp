/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CtrlSys/TrajectoryProfile.h"

#include <cmath>

using namespace frc;

/**
 * Constructs TrajectoryProfile
 */
TrajectoryProfile::TrajectoryProfile() {}

void TrajectoryProfile::SetTrajectory(std::unique_ptr<Segment[]>& trajectory) {
  m_trajectory = trajectory.get();
}
