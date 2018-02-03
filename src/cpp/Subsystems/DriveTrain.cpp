// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/DriveTrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>

DriveTrain::DriveTrain() {
    m_drive.SetDeadband(k_joystickDeadband);

    m_leftGrbx.SetSensorDirection(true);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_controller.GetPositionPID().SetPID(k_posP, k_posI, k_posD);
    m_controller.GetAnglePID().SetPID(k_angleP, k_angleI, k_angleD);

    m_controller.GetPositionPID().SetOutputRange(-0.25, 0.25);
    m_controller.GetAnglePID().SetOutputRange(-0.5, 0.5);

    m_controller.SetPositionTolerance(1.5,
                                      std::numeric_limits<double>::infinity());
    m_controller.SetAngleTolerance(1.5,
                                   std::numeric_limits<double>::infinity());
}

int32_t DriveTrain::GetLeftRaw() const { return m_leftGrbx.Get(); }

int32_t DriveTrain::GetRightRaw() const { return m_rightGrbx.Get(); }

void DriveTrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void DriveTrain::ResetEncoders() {
    m_leftGrbx.ResetEncoder();
    m_rightGrbx.ResetEncoder();
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

double DriveTrain::GetPosition() { return m_controller.GetPosition(); }

double DriveTrain::GetAngle() { return m_controller.GetAngle(); }

double DriveTrain::GetAngularRate() const { return m_gyro.GetRate(); }

void DriveTrain::StartClosedLoop() { m_controller.Enable(); }

void DriveTrain::StopClosedLoop() { m_controller.Disable(); }

void DriveTrain::SetPositionReference(double position) {
    m_posRef.Set(position);
}

void DriveTrain::SetAngleReference(double angle) { m_angleRef.Set(angle); }

double DriveTrain::GetPosReference() const { return m_posRef.GetOutput(); }

double DriveTrain::GetAngleReference() const { return m_angleRef.GetOutput(); }

bool DriveTrain::PosAtReference() const { return m_controller.AtPosition(); }

bool DriveTrain::AngleAtReference() const { return m_controller.AtAngle(); }

void DriveTrain::ResetGyro() { m_gyro.Reset(); }

void DriveTrain::CalibrateGyro() { m_gyro.Calibrate(); }

void DriveTrain::Debug() {}
