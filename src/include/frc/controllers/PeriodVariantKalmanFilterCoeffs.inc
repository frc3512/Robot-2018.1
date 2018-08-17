// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>::
    PeriodVariantKalmanFilterCoeffs(
        const Eigen::Matrix<double, States, States>& Qcontinuous,
        const Eigen::Matrix<double, Outputs, Outputs>& Rcontinuous,
        const Eigen::Matrix<double, States, States>& PsteadyState)
    : Qcontinuous(Qcontinuous),
      Rcontinuous(Rcontinuous),
      PsteadyState(PsteadyState) {}

template <int States, int Inputs, int Outputs>
PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>::
    PeriodVariantKalmanFilterCoeffs(const PeriodVariantKalmanFilterCoeffs& rhs)
    : Qcontinuous(rhs.Qcontinuous),
      Rcontinuous(rhs.Rcontinuous),
      PsteadyState(rhs.PsteadyState) {}

}  // namespace frc
