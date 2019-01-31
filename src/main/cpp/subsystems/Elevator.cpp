// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <limits>

#include <frc/DriverStation.h>

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

bool Elevator::GetBottomHallEffect() { return m_elevatorBottomHall.Get(); }

void Elevator::Enable() {
    m_controller.Enable();
    m_thread.StartPeriodic(0.005);
}

void Elevator::Disable() {
    m_controller.Disable();
    m_thread.Stop();
}

void Elevator::SetGoal(double position) { m_controller.SetGoal(position); }

bool Elevator::AtReference() const { return m_controller.AtReferences(); }

void Elevator::Iterate() {
    m_controller.SetMeasuredPosition(m_elevatorGearbox.GetPosition());
    m_controller.Update();

    // Set motor input
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_elevatorGearbox.Set(m_controller.ControllerVoltage() / batteryVoltage);
}

double Elevator::ControllerVoltage() const {
    return m_controller.ControllerVoltage();
}

void Elevator::Reset() { m_controller.Reset(); }

void Elevator::HandleEvent(Event event) {
    enum State { kPosition, kVelocity };
    static State state = State::kVelocity;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kPosition:
            if (event.type == EventType::kEntry) {
                Enable();
            }
            if (event == Event{kButtonPressed, 7}) {
                SetGoal(kFloorHeight);
            }
            if (event == Event{kButtonPressed, 8}) {
                SetGoal(kSwitchHeight);
            }
            if (event == Event{kButtonPressed, 9}) {
                SetGoal(kSecondBlockHeight);
            }
            if (event == Event{kButtonPressed, 10}) {
                SetGoal(kScaleHeight);
            }
            if (event == Event{kButtonPressed, 11}) {
                SetGoal(kClimbHeight);
            }
            if (event == Event{kButtonPressed, 12}) {
                nextState = State::kVelocity;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                SetGoal(GetHeight());
                Disable();
            }
            break;
        case State::kVelocity:
            SetVelocity(Robot::appendageStick.GetY());
            if (event == Event{kButtonPressed, 12}) {
                SetGoal(GetHeight());
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

void Elevator::Debug() {
    Robot::csvLogger.Log(m_controller.EstimatedPosition(),
                         m_controller.PositionReference(),
                         m_controller.ControllerVoltage());
}
