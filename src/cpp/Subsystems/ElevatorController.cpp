// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/ElevatorController.hpp"

#include <cmath>

ElevatorController::ElevatorController() { m_Y.setZero(); }

void ElevatorController::Enable() { m_loop.Enable(); }

void ElevatorController::Disable() { m_loop.Disable(); }

void ElevatorController::SetReferences(double position, double velocity) {
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << position, velocity;
    m_loop.SetNextR(nextR);
}

bool ElevatorController::AtReferences() const { return m_atReferences; }

void ElevatorController::SetMeasuredPosition(double measuredPosition) {
    m_Y << measuredPosition;
}

double ElevatorController::ControllerVoltage() const { return m_loop.U(0); }

double ElevatorController::EstimatedPosition() const { return m_loop.Xhat(0); }

double ElevatorController::EstimatedVelocity() const { return m_loop.Xhat(1); }

double ElevatorController::PositionError() const {
    return m_loop.Error()(0, 0);
}

double ElevatorController::VelocityError() const {
    return m_loop.Error()(1, 0);
}

void ElevatorController::Update() {
    m_loop.Correct(m_Y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_loop.Predict();
}

void ElevatorController::Reset() { m_loop.Reset(); }
