// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ADXRS450_Gyro.h>
#include <CtrlSys/FuncNode.h>
#include <CtrlSys/TrapezoidProfile.h>
#include <Drive/DifferentialDrive.h>
#include <Encoder.h>
#include <Notifier.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "ES/Service.hpp"
#include "Pose.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/DrivetrainController.hpp"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    DriveTrain();
    DriveTrain(const DriveTrain&) = delete;
    DriveTrain& operator=(const DriveTrain&) = delete;

    /**
     * Enable controller.
     */
    void Enable(void);
    /**
     * Disable controller.
     */
    void Disable(void);

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Set encoder distances to 0
    void ResetEncoders(void);

    /**
     * Sets the goal of the profiles
     *
     * @param pose  Target global pose x, y, and theta (not actually global yet)
     */
    void SetGoal(Pose pose);

    /*
     * Sets the references
     * 
     * @param leftPosition  Position of the left side in inches
     * @param leftVelocity  Velocity of the left side in inches per second
     * @param rightPosition Position of the right side in inches
     * @param rightVelocity Velocity of the right side in inches per second
     */
    void SetReferences(double leftPosition, double leftVelocity,
                      double rightPosition, double rightVelocity);

    bool AtReference() const;

    bool AtGoal() const;

    /**
     * Iterates the drivetrain control loop one cycle
     */
    void Iterate(void);

    /**
     * Returns controller output for left side
     */
    double ControllerLeftVoltage() const;
    /**
     * Returns controller output for right side
     */
    double ControllerRightVoltage() const;

    void Reset(void);

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
    void ResetGyro(void);

    // Calibrates gyro
    void CalibrateGyro(void);

    // Sends print statements for debugging purposes
    void Debug(void);

    void HandleEvent(Event event) override;

private:
    // Left gearbox used in position PID
    WPI_TalonSRX m_leftFront{kLeftDriveMasterID};
    WPI_TalonSRX m_leftRear{kLeftDriveSlaveID};
    CANTalonGroup m_leftGrbx{m_leftFront, m_leftRear};
    Encoder m_leftEncoder{kLeftEncoderA, kLeftEncoderB};

    // Right gearbox used in position PID
    WPI_TalonSRX m_rightFront{kRightDriveMasterID};
    WPI_TalonSRX m_rightRear{kRightDriveSlaveID};
    CANTalonGroup m_rightGrbx{m_rightFront, m_rightRear};
    Encoder m_rightEncoder{kRightEncoderA, kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    ADXRS450_Gyro m_gyro;
    DrivetrainController m_drivetrain;
    frc::Notifier m_thread{&DriveTrain::Iterate, this};
};
