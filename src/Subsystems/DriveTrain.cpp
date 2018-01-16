// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/DriveTrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>

using ctre::phoenix::motorcontrol::FeedbackDevice;

DriveTrain::DriveTrain() {
    m_drive.SetDeadband(k_joystickDeadband);

    m_rightGrbx.SetInverted(true);

    m_leftGrbx.SetSensorDirection(true);

    m_leftGrbx.SetFeedbackDevice(FeedbackDevice::QuadEncoder);
    m_rightGrbx.SetFeedbackDevice(FeedbackDevice::QuadEncoder);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
}

int32_t DriveTrain::GetLeftRaw() const { return m_leftGrbx.Get(); }

int32_t DriveTrain::GetRightRaw() const { return m_rightGrbx.Get(); }

void DriveTrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void DriveTrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void DriveTrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

void DriveTrain::StartClosedLoop() {}

void DriveTrain::StopClosedLoop() {}

void DriveTrain::Debug() {}
