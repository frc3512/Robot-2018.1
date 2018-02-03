// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Solenoid.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "ES/Service.hpp"

enum class MotorState { kIntake, kOuttake, kIdle };

class Intake : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    /**
     * Opens and closes the claw of the Intake
     */
    void ToggleOpen();

    /**
     * Stows and Deploys the Intake back into the frame of the elevator
     */
    void ToggleDeploy();

    bool GetDeploy();

    void SetMotors(MotorState state);

    void HandleEvent(Event event) override;

private:
    frc::Solenoid m_intakeClaw{kIntakeClawPort};
    frc::Solenoid m_intakeArm{kIntakeArmPort};
    WPI_TalonSRX m_intakeLeft{kIntakeLeftID};
    WPI_TalonSRX m_intakeRight{kIntakeRightID};
};
