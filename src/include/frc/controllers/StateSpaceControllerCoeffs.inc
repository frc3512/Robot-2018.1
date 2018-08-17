// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
StateSpaceControllerCoeffs<States, Inputs, Outputs>::StateSpaceControllerCoeffs(
    const Eigen::Matrix<double, Inputs, States>& K,
    const Eigen::Matrix<double, Inputs, States>& Kff,
    const Eigen::Matrix<double, Inputs, 1>& Umin,
    const Eigen::Matrix<double, Inputs, 1>& Umax)
    : K(K), Kff(Kff), Umin(Umin), Umax(Umax) {}

template <int States, int Inputs, int Outputs>
StateSpaceControllerCoeffs<States, Inputs, Outputs>::StateSpaceControllerCoeffs(
    const StateSpaceControllerCoeffs& rhs)
    : K(rhs.K), Kff(rhs.Kff), Umin(rhs.Umin), Umax(rhs.Umax) {}

}  // namespace frc
