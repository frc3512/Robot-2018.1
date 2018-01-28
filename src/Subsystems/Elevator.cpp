// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Elevator.hpp"

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.HandleEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

void Elevator::StartClosedLoop() { m_elevatorController.Enable(); }

void Elevator::StopClosedLoop() { m_elevatorController.Disable(); }

void Elevator::SetHeightReference(double height) {
    m_elevatorController.SetSetpoint(height);
}

double Elevator::GetHeightReference() const {
    return m_elevatorController.GetSetpoint();
}

bool Elevator::HeightAtReference() const {
    return m_elevatorController.OnTarget();
}

void Elevator::HandleEvent(Event event) {
    enum State { Idle, ClimberSetup, ClimberClimb };
    static State state = State::Idle;
    switch (state) {
        case State::Idle:
            if (event.type == EventType::kClimberSetup) {
                StartClosedLoop();
                SetHeightReference(k_climbHeight);
                m_notifier.StartPeriodic(0.05);
                state = State::ClimberSetup;
            }
            if (event.type == EventType::kClimberClimb) {
                StartClosedLoop();
                SetHeightReference(k_scaleHeight);
                m_notifier.StartPeriodic(0.05);
                state = State::ClimberClimb;
            }
            break;
        case State::ClimberSetup:
            if (HeightAtReference()) {
                m_notifier.Stop();
                Robot::climber.HandleEvent(EventType::kAtSetHeight);
                state = State::Idle;
            }
            break;
        case State::ClimberClimb:
            if (HeightAtReference()) {
                m_notifier.Stop();
                Robot::climber.HandleEvent(EventType::kAtSetHeight);
                state = State::Idle;
            }
            break;
    }
}
