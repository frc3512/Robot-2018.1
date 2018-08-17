/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

namespace frc {

/**
 * A container for all the observer coefficients.
 */
template <int States, int Inputs, int Outputs>
struct StateSpaceObserverCoeffs final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Estimator gain matrix.
   */
  const Eigen::Matrix<double, States, Outputs> L;

  /**
   * Construct the container for the observer coefficients.
   *
   * @param L The observer gain matrix.
   */
  explicit StateSpaceObserverCoeffs(
      const Eigen::Matrix<double, States, Outputs>& L);
};

}  // namespace frc

#include "frc/controllers/StateSpaceObserverCoeffs.inc"
