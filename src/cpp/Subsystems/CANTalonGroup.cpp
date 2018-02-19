// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/CANTalonGroup.hpp"

#include <memory>

#include <ctre/phoenix/MotorControl/SensorCollection.h>

using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

void CANTalonGroup::Set(double value) {
    if (m_forwardLimitSwitch != nullptr) {
        if (value > 0 && m_forwardLimitSwitch->Get() == m_limitPressedState) {
            value = 0.0;
        }
    }
    if (m_reverseLimitSwitch != nullptr) {
        if (value < 0 && m_reverseLimitSwitch->Get() == m_limitPressedState) {
            value = 0.0;
        }
    }
    if (value > 0 && GetPosition() > m_forwardLimit ||
        value < 0 && GetPosition() < m_reverseLimit) {
        value = 0.0;
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

void CANTalonGroup::EnableHardLimits(frc::DigitalInput* forwardLimitSwitch,
                                     frc::DigitalInput* reverseLimitSwitch) {
    m_forwardLimitSwitch = forwardLimitSwitch;
    m_reverseLimitSwitch = reverseLimitSwitch;
}

void CANTalonGroup::SetHardLimitPressedState(bool high) {
    m_limitPressedState = high;
}

void CANTalonGroup::EnableSoftLimits(double forwardLimit, double reverseLimit) {
    m_forwardLimit = forwardLimit;
    m_reverseLimit = reverseLimit;
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
