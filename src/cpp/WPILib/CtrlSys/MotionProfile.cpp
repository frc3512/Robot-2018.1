/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CtrlSys/MotionProfile.h"

#include <cmath>

using namespace frc;

MotionProfile::MotionProfile() { m_timer.Start(); }

double MotionProfile::GetOutput() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_ref = UpdateSetpoint(m_timer.Get());
  return std::get<0>(m_ref);
}

/**
 * Returns profile's goal state.
 */
double MotionProfile::GetGoal() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_goal;
}

/**
 * Returns true if motion profile has reached goal state.
 */
bool MotionProfile::AtGoal() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_lastTime >= m_timeTotal ||
         std::abs(m_goal - std::get<0>(m_ref)) < 0.001;
}

void MotionProfile::Reset() {
  m_lastTime = 0.0;
  m_timer.Reset();
}
