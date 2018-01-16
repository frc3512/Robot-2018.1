// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/DiffDriveController.h>
#include <CtrlSys/FuncNode.h>
#include <CtrlSys/RefInput.h>
#include <Drive/DifferentialDrive.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>

#include "Constants.hpp"
#include "Subsystems/CANTalonGroup.hpp"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain {
public:
    using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;

    DriveTrain();
    virtual ~DriveTrain() = default;

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Sends print statements for debugging purposes
    void Debug();

private:
    // Left gearbox used in position PID
    TalonSRX m_leftFront{k_leftDriveMasterID};
    TalonSRX m_leftRear{k_leftDriveSlaveID};
    CANTalonGroup m_leftGrbx{m_leftFront, m_leftRear};

    // Right gearbox used in position PID
    TalonSRX m_rightFront{k_rightDriveMasterID};
    TalonSRX m_rightRear{k_rightDriveSlaveID};
    CANTalonGroup m_rightGrbx{m_rightFront, m_rightRear};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};
};
