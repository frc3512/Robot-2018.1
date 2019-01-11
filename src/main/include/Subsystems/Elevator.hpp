// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/CtrlSys/FuncNode.h>
#include <frc/CtrlSys/Output.h>
#include <frc/CtrlSys/PIDNode.h>
#include <frc/CtrlSys/RefInput.h>
#include <frc/CtrlSys/SumNode.h>
#include <frc/DigitalInput.h>
#include <frc/Notifier.h>

#include "Constants.hpp"
#include "ES/Service.hpp"
#include "Subsystems/CANTalonGroup.hpp"

class Elevator : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    // Sets the voltage of the motors
    void SetVelocity(double velocity);

    // Set encoder distance to 0
    void ResetEncoder();

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Gets encoder values
    double GetHeight();

    // Sets encoder PID setpoints
    void SetHeightReference(double height);

    // Returns encoder PID loop references
    double GetHeightReference() const;

    // Returns whether or not elevator has reached reference
    bool HeightAtReference() const;

    // Gets whether the Hall Effect sensor has triggered
    bool GetBottomHallEffect();

    void HandleEvent(Event event) override;

private:
    WPI_TalonSRX m_elevatorMasterMotor{kElevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{kElevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    frc::Notifier m_notifier;
    // Reference
    frc::RefInput m_heightRef{0.0};

    // Sensors
    frc::DigitalInput m_elevatorBottomHall{kElevatorBottomHallPort};
    frc::FuncNode m_elevatorEncoder{
        [this] { return m_elevatorGearbox.GetPosition(); }};

    frc::RefInput m_feedForward{kGravityFeedForward};
    frc::SumNode m_errorSum{m_heightRef, true, m_elevatorEncoder, false};
    frc::PIDNode m_pid{kElevatorP,    kElevatorI, kElevatorD,
                       m_feedForward, m_errorSum, kElevatorControllerPeriod};
    frc::Output m_output{m_pid, m_elevatorGearbox, kElevatorControllerPeriod};
};
