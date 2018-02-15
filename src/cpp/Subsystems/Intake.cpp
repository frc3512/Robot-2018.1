// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Intake.hpp"

void Intake::Open() { m_intakeClaw.Set(true); }

void Intake::Close() { m_intakeClaw.Set(false); }

bool Intake::IsOpen() const { return m_intakeClaw.Get(); }

void Intake::Deploy() { m_intakeArm.Set(true); }

void Intake::Stow() { m_intakeArm.Set(false); }

bool Intake::IsDeployed() const { return m_intakeArm.Get(); }

void Intake::SetMotors(MotorState state) {
    // IntakeLeft is Inverted, IntakeRight is not
    if (state == MotorState::kIntake) {
        m_intakeLeft.Set(-1.0);
        m_intakeRight.Set(1.0);
    } else if (state == MotorState::kOuttake) {
        m_intakeLeft.Set(1.0);
        m_intakeRight.Set(-1.0);
    } else if (state == MotorState::kIdle) {
        m_intakeLeft.Set(0.0);
        m_intakeRight.Set(0.0);
    }
}

void Intake::HandleEvent(Event event) {
    if (event.type == EventType::kElevatorSetClimb) {
        m_intakeArm.Set(false);
    }
}
