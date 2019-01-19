// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/Notifier.h>
#include <frc/drive/DifferentialDrive.h>

#include "Constants.hpp"
#include "control/DrivetrainController.hpp"
#include "control/Pose.hpp"
#include "es/Service.hpp"
#include "subsystems/CANTalonGroup.hpp"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class Drivetrain : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Enable controller.
     */
    void Enable();
    /**
     * Disable controller.
     */
    void Disable();

    /**
     * Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Set encoder distances to 0
    void ResetEncoders();

    /**
     * Sets the goal of the profiles
     *
     * @param pose  Target global pose x, y, and theta (not actually global yet)
     */

    /**
     * Sets goal pose of drivetrain controller.
     *
     * @param pose The goal pose.
     */
    void SetGoal(const Pose& pose);

    /**
     * Sets goal pose of drivetrain controller.
     *
     * @param pose The goal pose.
     */
    void SetGoal(Pose&& pose);

    /**
     * Returns whether the drivetrain controller is at the goal pose.
     */
    bool AtGoal() const;

    /**
     * Iterates the drivetrain control loop one cycle
     */
    void Iterate();

    /**
     * Returns controller output for left side
     */
    double ControllerLeftVoltage() const;
    /**
     * Returns controller output for right side
     */
    double ControllerRightVoltage() const;

    void Reset();

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Returns encoder distances
    double GetLeftDisplacement() const;
    double GetRightDisplacement() const;

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    // Return gyro's rate
    double GetAngularRate() const;

    // Resets gyro
    void ResetGyro();

    // Calibrates gyro
    void CalibrateGyro();

    // Sends print statements for debugging purposes
    void Debug();

    void HandleEvent(Event event) override;

private:
    // Left gearbox used in position PID
    WPI_TalonSRX m_leftFront{kLeftDriveMasterID};
    WPI_TalonSRX m_leftRear{kLeftDriveSlaveID};
    CANTalonGroup m_leftGrbx{m_leftFront, m_leftRear};
    frc::Encoder m_leftEncoder{kLeftEncoderA, kLeftEncoderB};

    // Right gearbox used in position PID
    WPI_TalonSRX m_rightFront{kRightDriveMasterID};
    WPI_TalonSRX m_rightRear{kRightDriveSlaveID};
    CANTalonGroup m_rightGrbx{m_rightFront, m_rightRear};
    frc::Encoder m_rightEncoder{kRightEncoderA, kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;
    DrivetrainController m_controller;
    frc::Notifier m_thread{&Drivetrain::Iterate, this};
};
