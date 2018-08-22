// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/Notifier.h>

#include "Constants.hpp"
#include "Drivetrain.hpp"
#include "communications/PublishNode.hpp"
#include "control/ElevatorController.hpp"
#include "subsystems/CANTalonGroup.hpp"
#include "subsystems/SubsystemBase.hpp"

class Elevator : public SubsystemBase, public PublishNode {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    Elevator(const Elevator&) = delete;
    Elevator& operator=(const Elevator&) = delete;

    // Sets the voltage of the motors
    void SetVelocity(double velocity);

    // Set encoder distance to 0
    void ResetEncoder();

    // Gets encoder values
    double GetHeight();

    void Enable();
    void Disable();

    void SetGoal(double position);

    bool AtReference() const;

    void Iterate();

    double ControllerVoltage() const;

    void Reset();

    // Gets whether the Hall Effect sensor has triggered
    bool GetBottomHallEffect();

    void ProcessMessage(const ButtonPacket& message) override;

    void SubsystemPeriodic() override;

    void Debug();

private:
    WPI_TalonSRX m_elevatorMasterMotor{kElevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{kElevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    // Sensors
    frc::DigitalInput m_elevatorBottomHall{kElevatorBottomHallPort};

    ElevatorController m_controller;
    frc::Notifier m_thread{&Elevator::Iterate, this};
};
