// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/MotorControl/ControlMode.h>

using ctre::phoenix::motorcontrol::ControlMode;

template <class... CANTalons>
CANTalonGroup::CANTalonGroup(TalonSRX& canTalon, CANTalons&... canTalons)
    : CANTalonGroup(-1, -1, canTalon, canTalons...) {}

template <class... CANTalons>
CANTalonGroup::CANTalonGroup(int forwardLimitPin, int reverseLimitPin,
    TalonSRX& canTalon, CANTalons&... canTalons)
    : m_canTalons{canTalon, canTalons...} {
    if (forwardLimitPin != -1) {
        m_forwardLimit = std::make_unique<frc::DigitalInput>(forwardLimitPin);
    }

    if (reverseLimitPin != -1) {
        m_reverseLimit = std::make_unique<frc::DigitalInput>(reverseLimitPin);
    }

    for (size_t i = 0; i < m_canTalons.size(); i++) {
        if (i == 0) {
            m_canTalons[i].get().Set(ControlMode::PercentOutput, 0.0);
            m_canTalons[i].get().ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 50);
            m_canTalons[i].get().SetSensorPhase(m_isEncoderReversed);
            ResetEncoder();
        } else {
            // Set first CANTalon as master and all others as slaves
            m_canTalons[i].get().Set(ControlMode::Follower,
                                     m_canTalons[0].get().GetDeviceID());
        }
    }
}
