/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>

namespace frc {

template <int States, int Inputs, int Outputs>
StateSpaceObserver<States, Inputs, Outputs>::StateSpaceObserver(
    const StateSpaceObserverCoeffs<States, Inputs, Outputs>& observerCoeffs,
    StateSpacePlant<States, Inputs, Outputs>& plant)
    : m_plant(&plant) {
  AddCoefficients(observerCoeffs);
}

template <int States, int Inputs, int Outputs>
StateSpaceObserver<States, Inputs, Outputs>::StateSpaceObserver(
    StateSpaceObserver&& rhs) {
  std::swap(m_plant, rhs.m_plant);
  m_Xhat.swap(rhs.m_Xhat);
  std::swap(m_coefficients, rhs.m_coefficients);
  std::swap(m_index, rhs.m_index);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, Outputs>&
StateSpaceObserver<States, Inputs, Outputs>::L() const {
  return GetCoefficients().L;
}

template <int States, int Inputs, int Outputs>
double StateSpaceObserver<States, Inputs, Outputs>::L(int i, int j) const {
  return L()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
StateSpaceObserver<States, Inputs, Outputs>::Xhat() const {
  return m_Xhat;
}

template <int States, int Inputs, int Outputs>
double StateSpaceObserver<States, Inputs, Outputs>::Xhat(int i) const {
  return Xhat()(i, 0);
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::SetXhat(
    const Eigen::Matrix<double, States, 1>& xHat) {
  m_Xhat = xHat;
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::SetXhat(int i, double value) {
  m_Xhat(i, 0) = value;
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::Reset() {
  m_Xhat.setZero();
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::Predict(
    const Eigen::Matrix<double, Inputs, 1>& newU) {
  m_Xhat = m_plant->UpdateX(Xhat(), newU);
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::Correct(
    const Eigen::Matrix<double, Inputs, 1>& u,
    const Eigen::Matrix<double, Outputs, 1>& y) {
  /*
   * A^-1 is applied here to step the L matrix back an iteration to match the
   * prediction step. Note that this cancels the A in "A * K" for a Kalman
   * filter run as a Luenberger observer to give simply K, as expected for
   * separate steps.
   */
  m_Xhat +=
      m_plant->Ainv() * L() * (y - m_plant->C() * Xhat() - m_plant->D() * u);
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::AddCoefficients(
    const StateSpaceObserverCoeffs<States, Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const StateSpaceObserverCoeffs<States, Inputs, Outputs>&
StateSpaceObserver<States, Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const StateSpaceObserverCoeffs<States, Inputs, Outputs>&
StateSpaceObserver<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void StateSpaceObserver<States, Inputs, Outputs>::SetIndex(int index) {
  if (index < 0) {
    m_index = 0;
  } else if (index >= static_cast<int>(m_coefficients.size())) {
    m_index = static_cast<int>(m_coefficients.size()) - 1;
  } else {
    m_index = index;
  }
}

template <int States, int Inputs, int Outputs>
int StateSpaceObserver<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

}  // namespace frc
