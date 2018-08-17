// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

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
