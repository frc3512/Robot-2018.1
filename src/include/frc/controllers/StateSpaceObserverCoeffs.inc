// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
StateSpaceObserverCoeffs<States, Inputs, Outputs>::StateSpaceObserverCoeffs(
    const Eigen::Matrix<double, States, Outputs>& L)
    : L(L) {}

}  // namespace frc
