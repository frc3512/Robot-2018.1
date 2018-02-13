// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/MotorControl/ControlMode.h>
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>

template <class... CANTalons>
CANTalonGroup::CANTalonGroup(WPI_TalonSRX& canTalon, CANTalons&... canTalons)
    : m_canTalons{canTalon, canTalons...} {
    using ctre::phoenix::motorcontrol::ControlMode;
    for (size_t i = 0; i < m_canTalons.size(); i++) {
        if (i == 0) {
            using FeedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice;
            m_canTalons[i].get().Set(ControlMode::PercentOutput, 0.0);
            m_canTalons[i].get().ConfigSelectedFeedbackSensor(
                FeedbackDevice::QuadEncoder, 0, 0);
            m_canTalons[i].get().SetSensorPhase(false);
            ResetEncoder();
        } else {
            // Set first CANTalon as master and all others as slaves
            m_canTalons[i].get().Follow(m_canTalons[0]);
        }
    }
}
