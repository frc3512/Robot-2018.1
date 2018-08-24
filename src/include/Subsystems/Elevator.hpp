// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/FuncNode.h>
#include <CtrlSys/Output.h>
#include <CtrlSys/PIDNode.h>
#include <CtrlSys/RefInput.h>
#include <CtrlSys/SumNode.h>
#include <DigitalInput.h>
#include <DriverStation.h>
#include <Notifier.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DriveTrain.hpp"
#include "ES/Service.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/ElevatorController.hpp"

class Elevator : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    Elevator(const Elevator&) = delete;
    Elevator& operator=(const Elevator&) = delete;

    // Sets the voltage of the motors
    void SetVelocity(double velocity);

    // Set encoder distance to 0
    void ResetEncoder(void);

    // Gets encoder values
    double GetHeight(void);

    void Enable(void);
    void Disable(void);

    void SetReferences(double position, double velocity);

    bool AtReference() const;

    void Iterate(void);

    double ControllerVoltage() const;

    void Reset(void);

    // Gets whether the Hall Effect sensor has triggered
    bool GetBottomHallEffect(void);

    void HandleEvent(Event event) override;

private:
    WPI_TalonSRX m_elevatorMasterMotor{kElevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{kElevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    Notifier m_notifier;
    // Reference
    frc::RefInput m_heightRef{0.0};

    // Sensors
    frc::DigitalInput m_elevatorBottomHall{kElevatorBottomHallPort};
    frc::FuncNode m_elevatorEncoder{
        [this] { return m_elevatorGearbox.GetPosition(); }};

    ElevatorController m_elevator;
    frc::Notifier m_thread{&Elevator::Iterate, this};
};
