// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>

namespace frc {

/**
 * A container for all the observer coefficients.
 */
template <int States, int Inputs, int Outputs>
struct PeriodVariantKalmanFilterCoeffs final {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Process noise covariance matrix (continuous).
     */
    const Eigen::Matrix<double, States, States> Qcontinuous;

    /**
     * Measurement noise covariance matrix (continuous).
     */
    const Eigen::Matrix<double, Outputs, Outputs> Rcontinuous;

    /**
     * Steady-state error covariance matrix (continuous).
     */
    const Eigen::Matrix<double, States, States> PsteadyState;

    /**
     * Construct the container for the observer coefficients.
     *
     * @param Qcontinuous  Continuous process noise covariance matrix.
     * @param Rcontinuous  Continuous measurement noise covariance matrix.
     * @param PsteadyState Continuous steady-state error covariance matrix.
     */
    PeriodVariantKalmanFilterCoeffs(
        const Eigen::Matrix<double, States, States>& Qcontinuous,
        const Eigen::Matrix<double, Outputs, Outputs>& Rcontinuous,
        const Eigen::Matrix<double, States, States>& PsteadyState);

    PeriodVariantKalmanFilterCoeffs(const PeriodVariantKalmanFilterCoeffs& rhs);
};

}  // namespace frc

#include "frc/controllers/PeriodVariantKalmanFilterCoeffs.inc"
