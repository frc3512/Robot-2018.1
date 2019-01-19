// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

Drivetrain::Drivetrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftEncoder.SetDistancePerPulse(kDpP);
    m_rightEncoder.SetDistancePerPulse(kDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_drive.SetRightSideInverted(false);
    m_leftGrbx.SetInverted(true);
    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
}

/**
 * Enable controller.
 */
void Drivetrain::Enable() {
    m_thread.StartPeriodic(kDt);
    m_controller.Enable();
    m_drive.SetSafetyEnabled(false);
}

void Drivetrain::Disable() {
    m_controller.Disable();
    m_thread.Stop();
    m_drive.SetSafetyEnabled(true);
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(-throttle, turn, isQuickTurn);
}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::SetGoal(const Pose& pose) { m_controller.SetGoal(pose); }

void Drivetrain::SetGoal(Pose&& pose) { m_controller.SetGoal(std::move(pose)); }

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

void Drivetrain::Iterate() {
    m_controller.SetMeasuredStates(m_leftEncoder.GetRate(),
                                   m_rightEncoder.GetRate(), m_gyro.GetAngle());
    m_controller.Update();

    // Set motor inputs
    double batteryVoltage =
        frc::DriverStation::GetInstance().GetBatteryVoltage();
    m_leftGrbx.Set(m_controller.ControllerLeftVoltage() / batteryVoltage / 2);
    m_rightGrbx.Set(m_controller.ControllerRightVoltage() / batteryVoltage / 2);
}

double Drivetrain::ControllerLeftVoltage() const {
    return m_controller.ControllerLeftVoltage();
}

double Drivetrain::ControllerRightVoltage() const {
    return m_controller.ControllerRightVoltage();
}

void Drivetrain::Reset() { m_controller.Reset(); }

int32_t Drivetrain::GetLeftRaw() const { return m_leftEncoder.Get(); }

int32_t Drivetrain::GetRightRaw() const { return m_rightEncoder.Get(); }

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double Drivetrain::GetLeftDisplacement() const {
    return m_leftGrbx.GetPosition();
}

double Drivetrain::GetRightDisplacement() const {
    return m_rightGrbx.GetPosition();
}

double Drivetrain::GetLeftRate() const { return m_leftGrbx.GetSpeed(); }

double Drivetrain::GetRightRate() const { return m_rightGrbx.GetSpeed(); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

void Drivetrain::Debug() {
    TrajectoryPoint estimatedPose = m_controller.EstimatedPose();
    TrajectoryPoint desiredPose = m_controller.GoalPose();
    /*
    Robot::logger.Log(
        LogEvent("Estimated Pose: " + std::to_string(estimatedPose.x) + " , " +
                     std::to_string(estimatedPose.y) + " , " +
                     std::to_string(estimatedPose.theta),
                 LogEvent::VERBOSE_DEBUG));
    Robot::logger.Log(LogEvent(
        "Left Error: " + std::to_string(m_controller.LeftVelocityError()) +
            " Right Error: " +
            std::to_string(m_controller.RightVelocityError()),
        LogEvent::VERBOSE_DEBUG));
    Robot::logger.Log(LogEvent(
        "Left U: " + std::to_string(m_controller.ControllerLeftVoltage()) +
            " Right U: " +
            std::to_string(m_controller.ControllerRightVoltage()),
        LogEvent::VERBOSE_DEBUG));
    Robot::logger.Log(LogEvent(
        "Left Pos: " + std::to_string(m_leftEncoder.GetDistance()) +
            " Right Pos: " + std::to_string(m_rightEncoder.GetDistance()) +
            "Angle" + std::to_string(deg2rad(m_gyro.GetAngle())),
        LogEvent::VERBOSE_DEBUG));*/
    Robot::csvLogger.Log(estimatedPose.x, estimatedPose.y, estimatedPose.theta,
                         desiredPose.x, desiredPose.y, desiredPose.theta,
                         m_controller.ControllerLeftVoltage(),
                         m_controller.ControllerRightVoltage(),
                         m_controller.LeftVelocityReference(),
                         m_controller.RightVelocityReference(),
                         m_controller.EstimatedLeftVelocity(),
                         m_controller.EstimatedRightVelocity());
}

void Drivetrain::HandleEvent(Event event) {}
