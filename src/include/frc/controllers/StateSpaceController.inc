// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace frc {

template <int States, int Inputs, int Outputs>
StateSpaceController<States, Inputs, Outputs>::StateSpaceController(
    const StateSpaceControllerCoeffs<States, Inputs, Outputs>& controllerCoeffs,
    StateSpacePlant<States, Inputs, Outputs>& plant)
    : m_plant(&plant) {
    AddCoefficients(controllerCoeffs);
}

template <int States, int Inputs, int Outputs>
StateSpaceController<States, Inputs, Outputs>::StateSpaceController(
    StateSpaceController&& rhs) {
    std::swap(m_plant, rhs.m_plant);
    std::swap(m_enabled, rhs.m_enabled);
    m_R.swap(rhs.m_R);
    m_U.swap(rhs.m_U);
    std::swap(m_coefficients, rhs.m_coefficients);
    std::swap(m_index, rhs.m_index);
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::Enable() {
    m_enabled = true;
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::Disable() {
    m_enabled = false;
    m_U.setZero();
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, States>&
StateSpaceController<States, Inputs, Outputs>::K() const {
    return GetCoefficients().K;
}

template <int States, int Inputs, int Outputs>
double StateSpaceController<States, Inputs, Outputs>::K(int i, int j) const {
    return K()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, States>&
StateSpaceController<States, Inputs, Outputs>::Kff() const {
    return GetCoefficients().Kff;
}

template <int States, int Inputs, int Outputs>
double StateSpaceController<States, Inputs, Outputs>::Kff(int i, int j) const {
    return Kff()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
StateSpaceController<States, Inputs, Outputs>::R() const {
    return m_R;
}

template <int States, int Inputs, int Outputs>
double StateSpaceController<States, Inputs, Outputs>::R(int i) const {
    return R()(i, 0);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>&
StateSpaceController<States, Inputs, Outputs>::U() const {
    return m_U;
}

template <int States, int Inputs, int Outputs>
double StateSpaceController<States, Inputs, Outputs>::U(int i) const {
    return U()(i, 0);
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::Reset() {
    m_R.setZero();
    m_U.setZero();
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::Update(
    const Eigen::Matrix<double, States, 1>& x) {
    if (m_enabled) {
        m_U = K() * (m_R - x) + Kff() * (m_R - m_plant->A() * m_R);
        CapU();
    }
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::Update(
    const Eigen::Matrix<double, States, 1>& nextR,
    const Eigen::Matrix<double, States, 1>& x) {
    if (m_enabled) {
        m_U = K() * (m_R - x) + Kff() * (nextR - m_plant->A() * m_R);
        CapU();
        m_R = nextR;
    }
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::CapU() {
    for (int i = 0; i < Inputs; ++i) {
        if (U(i) > GetCoefficients().Umax(i)) {
            m_U(i, 0) = GetCoefficients().Umax(i);
        } else if (U(i) < GetCoefficients().Umin(i)) {
            m_U(i, 0) = GetCoefficients().Umin(i);
        }
    }
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::AddCoefficients(
    const StateSpaceControllerCoeffs<States, Inputs, Outputs>& coefficients) {
    m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const StateSpaceControllerCoeffs<States, Inputs, Outputs>&
StateSpaceController<States, Inputs, Outputs>::GetCoefficients(
    int index) const {
    return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const StateSpaceControllerCoeffs<States, Inputs, Outputs>&
StateSpaceController<States, Inputs, Outputs>::GetCoefficients() const {
    return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void StateSpaceController<States, Inputs, Outputs>::SetIndex(int index) {
    if (index < 0) {
        m_index = 0;
    } else if (index >= static_cast<int>(m_coefficients.size())) {
        m_index = static_cast<int>(m_coefficients.size()) - 1;
    } else {
        m_index = index;
    }
}

template <int States, int Inputs, int Outputs>
int StateSpaceController<States, Inputs, Outputs>::GetIndex() const {
    return m_index;
}

}  // namespace frc