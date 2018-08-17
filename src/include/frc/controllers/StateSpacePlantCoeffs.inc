/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
StateSpacePlantCoeffs<States, Inputs, Outputs>::StateSpacePlantCoeffs(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, States>& Ainv,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, Outputs, States>& C,
    const Eigen::Matrix<double, Outputs, Inputs>& D)
    : A(A), Ainv(Ainv), B(B), C(C), D(D) {}

template <int States, int Inputs, int Outputs>
StateSpacePlantCoeffs<States, Inputs, Outputs>::StateSpacePlantCoeffs(
    const StateSpacePlantCoeffs& rhs)
    : A(rhs.A), Ainv(rhs.Ainv), B(rhs.B), C(rhs.C), D(rhs.D) {}

}  // namespace frc
