// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/DriveTrain.hpp"
#include "Robot.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include <DriverStation.h>

DriveTrain::DriveTrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftEncoder.SetDistancePerPulse(kDpP);
    m_rightEncoder.SetDistancePerPulse(kDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
}

/**
* Enable controller.
*/
void DriveTrain::Enable() {
    m_thread.StartPeriodic(0.005);
}

void DriveTrain::Disable() {
    m_drivetrain.Disable();
    m_thread.Stop();
}

void DriveTrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void DriveTrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void DriveTrain::SetReferences(double leftPosition, double leftVelocity, double rightPosition, double rightVelocity){
    m_drivetrain.SetReferences(leftPosition, leftVelocity, rightPosition, rightVelocity);
}

bool DriveTrain::AtReference() const {
    return m_drivetrain.AtReferences();
}

void DriveTrain::Iterate() {
    m_drivetrain.SetMeasuredStates(m_leftEncoder.Get(), m_rightEncoder.Get());
    m_drivetrain.Update();

    // Set motor inputs
    double batteryVoltage = frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_leftGrbx.Set(m_drivetrain.ControllerLeftVoltage() / batteryVoltage);
    m_rightGrbx.Set(m_drivetrain.ControllerRightVoltage() / batteryVoltage);
}

double DriveTrain::ControllerLeftVoltage() const{
    return m_drivetrain.ControllerLeftVoltage();
}

double DriveTrain::ControllerRightVoltage() const {
    return m_drivetrain.ControllerRightVoltage();
}

void DriveTrain::Reset() {
    m_drivetrain.Reset();
}

int32_t DriveTrain::GetLeftRaw() const {
    return m_leftEncoder.Get();
}

int32_t DriveTrain::GetRightRaw() const {
    return m_rightEncoder.Get();
}

void DriveTrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void DriveTrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double DriveTrain::GetLeftDisplacement() const {
  return m_leftGrbx.GetPosition();
 }

double DriveTrain::GetRightDisplacement() const {
    return m_rightGrbx.GetPosition();
}

double DriveTrain::GetLeftRate() const { return m_leftGrbx.GetSpeed(); }

double DriveTrain::GetRightRate() const { return m_rightGrbx.GetSpeed(); }

double DriveTrain::GetAngularRate() const { return m_gyro.GetRate(); }

void DriveTrain::ResetGyro() { m_gyro.Reset(); }

void DriveTrain::CalibrateGyro() { m_gyro.Calibrate(); }

void DriveTrain::Debug() {
    Robot::logger.Log(LogEvent(
        "Left Pos: " + std::to_string(m_leftEncoder.GetDistance()) +
            " Right Pos: " + std::to_string(m_rightEncoder.GetDistance()),
        LogEvent::VERBOSE_DEBUG));
}

void DriveTrain::HandleEvent(Event event) {}
