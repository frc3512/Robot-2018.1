// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/FuncNode.h>
#include <CtrlSys/PIDController.h>
#include <CtrlSys/RefInput.h>
#include <DigitalInput.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "Subsystems/CANTalonGroup.hpp"

class Elevator {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    void SetVelocity(double velocity);

    // Set encoder distance to 0
    void ResetEncoder();

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Sets encoder PID setpoints
    void SetHeightReference(double height);

    // Returns encoder PID loop references
    double GetHeightReference() const;

    // Returns whether or not elevator has reached reference
    bool HeightAtReference() const;

private:
    WPI_TalonSRX m_elevatorMasterMotor{k_elevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{k_elevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    // Reference
    frc::RefInput m_heightRef{0.0};

    // Sensors
    frc::DigitalInput m_elevatorHallEffect{k_elevatorHallPort};
    frc::FuncNode m_elevatorEncoder{
        [this] { return m_elevatorGearbox.GetPosition(); }};

    frc::PIDController m_elevatorController{
        k_elevatorP,       k_elevatorI,       k_elevatorD,
        m_elevatorEncoder, m_elevatorGearbox, k_elevatorControllerPeriod};
};
