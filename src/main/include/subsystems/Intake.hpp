// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/Solenoid.h>

#include "Constants.hpp"
#include "es/Service.hpp"

enum class MotorState { kIntake, kOuttake, kIdle };

class Intake : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    /**
     * Opens and closes the claw of the Intake, also gets the current state of
     * the Intake
     */
    void Open();

    void Close();

    bool IsOpen() const;
    /**
     * Stows and Deploys the Intake back into the frame of the elevator, also
     * gets the current state of the Intake
     */
    void Deploy();

    void Stow();

    bool IsDeployed() const;

    void SetMotors(MotorState state);

    void AutoOuttake();

    void HandleEvent(Event event) override;

private:
    frc::Solenoid m_intakeClaw{kIntakeClawPort};
    frc::Solenoid m_intakeArm{kIntakeArmPort};
    WPI_TalonSRX m_intakeLeft{kIntakeLeftID};
    WPI_TalonSRX m_intakeRight{kIntakeRightID};
};
