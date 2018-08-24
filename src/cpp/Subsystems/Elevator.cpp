// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

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

bool Elevator::GetBottomHallEffect() { return m_elevatorBottomHall.Get(); }

void Elevator::Enable() {
  m_elevator.Enable();
  m_thread.StartPeriodic(0.005);
}

void Elevator::Disable() {
  m_elevator.Disable();
  m_thread.Stop();
}

void Elevator::SetReferences(double position, double velocity) {
  m_elevator.SetReferences(position, velocity);
}

void Elevator::Iterate() {
  m_elevator.SetMeasuredPosition(m_encoder.GetDistance());
  m_elevator.Update();

  // Set motor input
  double batteryVoltage = frc::DriverStation::GetInstance().GetBatteryVoltage();
  m_motor.Set(m_elevator.ControllerVoltage() / batteryVoltage);
}

double Elevator::ControllerVoltage() const {
  return m_elevator.ControllerVoltage();
}

void Elevator::Reset() { m_elevator.Reset(); }

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
