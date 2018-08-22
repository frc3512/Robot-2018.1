// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Elevator.hpp"

#include <limits>

#include <frc/DriverStation.h>

#include "Robot.hpp"

Elevator::Elevator() : PublishNode("Elevator") {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
    m_elevatorGearbox.EnableHardLimits(&m_elevatorBottomHall, nullptr);
    m_elevatorGearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                                       kClimbHeight);
    m_elevatorGearbox.SetHardLimitPressedState(false);
    Subscribe(*this);
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

enum class State { kPosition, kVelocity };
static State state = State::kVelocity;

void Elevator::ProcessMessage(const ButtonPacket& message) {
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kPosition:
            if (message.topic == "Event/StateEntry") {
                Enable();
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 7 && message.pressed) {
                SetGoal(kFloorHeight);
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 8 && message.pressed) {
                SetGoal(kSwitchHeight);
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 9 && message.pressed) {
                SetGoal(kSecondBlockHeight);
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 10 && message.pressed) {
                SetGoal(kScaleHeight);
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 11 && message.pressed) {
                SetGoal(kClimbHeight);
            }
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 12 && message.pressed) {
                nextState = State::kVelocity;
                makeTransition = true;
            }
            if (message.topic == "Event/StateExit") {
                SetGoal(GetHeight());
                Disable();
            }
            break;
        case State::kVelocity:
            if (message.topic == "Robot/AppendageStick" &&
                message.button == 12 && message.pressed) {
                SetGoal(GetHeight());
                nextState = State::kPosition;
                makeTransition = true;
            }
            break;
    }
    if (makeTransition) {
        ButtonPacket stateExit{"Event/StateExit", 0, false};
        ProcessMessage(stateExit);
        state = nextState;
        ButtonPacket stateEntry{"Event/StateEntry", 0, false};
        ProcessMessage(stateEntry);
    }
}

void Elevator::SubsystemPeriodic() {
    switch (state) {
        case State::kVelocity:
            SetVelocity(Robot::appendageStick.GetY());
            break;
        case State::kPosition:
            break;
    }
}

void Elevator::Debug() {
    Robot::csvLogger.Log(m_controller.EstimatedPosition(),
                         m_controller.PositionReference(),
                         m_controller.ControllerVoltage());
}
