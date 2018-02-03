// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/CANTalonGroup.hpp"

#include <memory>

#include <ctre/phoenix/MotorControl/SensorCollection.h>

using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

void CANTalonGroup::Set(double value) {
    if (m_forwardLimit != nullptr && m_reverseLimit != nullptr) {
        if (value > 0 && m_forwardLimit->Get() == m_limitPressedState) {
            value = 0.0;
        } else if (value < 0 && m_reverseLimit->Get() == m_limitPressedState) {
            value = 0.0;
        }
    }
    m_canTalons[0].get().Set(ControlMode::PercentOutput, value);
}

double CANTalonGroup::Get() const { return m_canTalons[0].get().Get(); }

void CANTalonGroup::SetInverted(bool isInverted) {
    for (auto& canTalon : m_canTalons) {
        canTalon.get().SetInverted(isInverted);
    }
}

bool CANTalonGroup::GetInverted() const {
    return m_canTalons[0].get().GetInverted();
}

void CANTalonGroup::Disable() {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, 0.0);
}

void CANTalonGroup::StopMotor() { Disable(); }

void CANTalonGroup::PIDWrite(double output) { Set(output); }

void CANTalonGroup::EnableHardLimits(int forwardLimitPin, int reverseLimitPin) {
    m_forwardLimit = std::make_unique<frc::DigitalInput>(forwardLimitPin);
    m_reverseLimit = std::make_unique<frc::DigitalInput>(reverseLimitPin);
}

void CANTalonGroup::SetLimitPressedState(bool high) {
    m_limitPressedState = high;
}

double CANTalonGroup::GetPosition() const {
    return m_canTalons[0].get().GetSelectedSensorPosition(0) *
           m_distancePerPulse;
}

double CANTalonGroup::GetSpeed() const {
    // RPM * degrees/rev / (seconds/min)
    return m_canTalons[0].get().GetSelectedSensorVelocity(0) *
           m_distancePerPulse / 60.0;
}

void CANTalonGroup::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void CANTalonGroup::ResetEncoder() {
    m_canTalons[0].get().GetSensorCollection().SetQuadraturePosition(0, 0);
}

void CANTalonGroup::SetSensorDirection(bool reverse) {
    m_canTalons[0].get().SetSensorPhase(reverse);
}

double CANTalonGroup::GetOutput() { return GetPosition(); }
