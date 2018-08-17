// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
PeriodVariantPlantCoeffs<States, Inputs, Outputs>::PeriodVariantPlantCoeffs(
    const Eigen::Matrix<double, States, States>& Acontinuous,
    const Eigen::Matrix<double, States, Inputs>& Bcontinuous,
    const Eigen::Matrix<double, Outputs, States>& C,
    const Eigen::Matrix<double, Outputs, Inputs>& D)
    : Acontinuous(Acontinuous), Bcontinuous(Bcontinuous), C(C), D(D) {}

template <int States, int Inputs, int Outputs>
PeriodVariantPlantCoeffs<States, Inputs, Outputs>::PeriodVariantPlantCoeffs(
    const PeriodVariantPlantCoeffs& rhs)
    : Acontinuous(rhs.Acontinuous),
      Bcontinuous(rhs.Bcontinuous),
      C(rhs.C),
      D(rhs.D) {}

}  // namespace frc
