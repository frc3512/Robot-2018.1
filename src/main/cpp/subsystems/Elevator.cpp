// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <limits>

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
    m_elevatorGearbox.EnableHardLimits(&m_elevatorBottomHall, nullptr);
    m_elevatorGearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                                       kClimbHeight);
    m_elevatorGearbox.SetHardLimitPressedState(false);
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { return m_elevatorGearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_output.Enable(); }

void Elevator::StopClosedLoop() { m_output.Disable(); }

void Elevator::SetHeightReference(double height) { m_heightRef.Set(height); }

double Elevator::GetHeightReference() const { return m_heightRef.GetOutput(); }

bool Elevator::HeightAtReference() const { return m_errorSum.InTolerance(); }

bool Elevator::GetBottomHallEffect() { return m_elevatorBottomHall.Get(); }

void Elevator::HandleEvent(Event event) {
    enum State { kPosition, kVelocity };
    static State state = State::kVelocity;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kPosition:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
            }
            if (event == Event{kButtonPressed, 7}) {
                SetHeightReference(kFloorHeight);
            }
            if (event == Event{kButtonPressed, 8}) {
                SetHeightReference(kSwitchHeight);
            }
            if (event == Event{kButtonPressed, 9}) {
                SetHeightReference(kSecondBlockHeight);
            }
            if (event == Event{kButtonPressed, 10}) {
                SetHeightReference(kScaleHeight);
            }
            if (event == Event{kButtonPressed, 11}) {
                SetHeightReference(kClimbHeight);
            }
            if (event == Event{kButtonPressed, 12}) {
                nextState = State::kVelocity;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                SetHeightReference(GetHeight());
                StopClosedLoop();
            }
            break;
        case State::kVelocity:
            SetVelocity(Robot::appendageStick.GetY());
            if (event == Event{kButtonPressed, 12}) {
                SetHeightReference(GetHeight());
                nextState = State::kPosition;
                makeTransition = true;
            }
            break;
    }
    if (makeTransition) {
        HandleEvent(EventType::kExit);
        state = nextState;
        HandleEvent(EventType::kEntry);
    }
}
