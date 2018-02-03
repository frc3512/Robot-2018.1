// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { m_elevatorGearbox.GetPosition(); }

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

bool Elevator::GetHallEffect() { return m_elevatorHallEffect.Get(); }

void Elevator::HandleEvent(Event event) {
    enum State {
        kIdle,
        kElevatorClimb,
        kElevatorScale,
        kElevatorSwitch,
    };
    static State state = State::kIdle;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kIdle:
            if (event.type == EventType::kElevatorSetSwitch) {
                nextState = State::kElevatorSwitch;
                makeTransition = true;
            } else if (event.type == EventType::kElevatorSetScale) {
                nextState = State::kElevatorScale;
                makeTransition = true;
            } else if (event.type == EventType::kElevatorSetClimb) {
                nextState = State::kElevatorClimb;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.StartPeriodic(0.05);
            }
            break;
        case State::kElevatorSwitch:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kSwitchHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::kElevatorScale:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kScaleHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::kElevatorClimb:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kClimbHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
            if (makeTransition) {
                PostEvent(EventType::kExit);
                state = nextState;
                PostEvent(EventType::kEntry);
            }
    }
}
