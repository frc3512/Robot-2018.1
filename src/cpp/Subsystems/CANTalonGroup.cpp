// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/CANTalonGroup.hpp"

#include <memory>

#include <ctre/phoenix/MotorControl/SensorCollection.h>

using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;

void CANTalonGroup::Set(double value) {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, value);
}

double CANTalonGroup::Get() const {
    return m_canTalons[0].get().GetSelectedSensorPosition(
        m_canTalons[0].get().GetDeviceID());
}

void CANTalonGroup::SetInverted(bool isInverted) {
    m_canTalons[0].get().SetInverted(isInverted);
}

bool CANTalonGroup::GetInverted() const {
    return m_canTalons[0].get().GetInverted();
}

void CANTalonGroup::Disable() {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, 0.0);
}

void CANTalonGroup::StopMotor() { Disable(); }

void CANTalonGroup::PIDWrite(double output) {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, output);
}

double CANTalonGroup::GetPosition() const {
    return m_canTalons[0].get().GetSelectedSensorPosition(
               m_canTalons[0].get().GetDeviceID()) *
           m_distancePerPulse;
}

double CANTalonGroup::GetSpeed() const {
    // RPM * degrees/rev / (seconds/min)
    return m_canTalons[0].get().GetSelectedSensorVelocity(
               m_canTalons[0].get().GetDeviceID()) *
           m_distancePerPulse / 60.0;
}

void CANTalonGroup::SetDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
}

void CANTalonGroup::SetFeedbackDevice(FeedbackDevice device) {
    m_feedbackDevice = device;
    m_canTalons[0].get().ConfigSelectedFeedbackSensor(device, 0, 0);
}

void CANTalonGroup::ResetEncoder() {
    m_canTalons[0].get().GetSensorCollection().SetQuadraturePosition(0, 0);
}

void CANTalonGroup::SetSensorDirection(bool reverse) {
    m_isEncoderReversed = reverse;
    m_canTalons[0].get().SetSensorPhase(m_isEncoderReversed);
}

bool CANTalonGroup::IsEncoderReversed() const { return m_isEncoderReversed; }

void CANTalonGroup::SetLimitOnHigh(bool limitOnHigh) {
    m_limitOnHigh = limitOnHigh;
}

void CANTalonGroup::SetSoftPositionLimits(double min, double max) {
    m_min = min;
    m_max = max;
}

TalonSRX& CANTalonGroup::GetMaster() { return m_canTalons[0].get(); }

double CANTalonGroup::GetOutput() { return GetPosition(); }
