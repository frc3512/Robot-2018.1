/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>
#include <mutex>
#include <tuple>

#include "CtrlSys/INode.h"
#include "Timer.h"

namespace frc {

/**
 * Base class for all types of motion profile controllers.
 */
class MotionProfile : public INode {
 public:
  MotionProfile();
  virtual ~MotionProfile() = default;

  double GetOutput() override;

  virtual void SetGoal(double goal, double currentSource) = 0;
  double GetGoal() const;
  bool AtGoal() const;

  void Reset();

 protected:
  using State = std::tuple<double, double, double>;

  virtual State UpdateSetpoint(double currentTime) = 0;

  // Use this to make UpdateSetpoint() and SetGoal() thread-safe
  mutable std::mutex m_mutex;

  Timer m_timer;

  double m_goal = 0.0;

  // Current reference (displacement, velocity, acceleration)
  State m_ref = std::make_tuple(0.0, 0.0, 0.0);

  double m_lastTime = 0.0;
  double m_timeTotal = std::numeric_limits<double>::infinity();
};

}  // namespace frc
