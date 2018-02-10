// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Intake.hpp"

void Intake::ToggleOpen() { m_intakeClaw.Set(!m_intakeClaw.Get()); }

void Intake::ToggleDeploy() { m_intakeArm.Set(!m_intakeArm.Get()); }

void Intake::SetMotors(MotorState state) {
    // IntakeLeft is Inverted, IntakeRight is not
    if (state == MotorState::k_intake) {
        m_intakeLeft.Set(-1.0);
        m_intakeRight.Set(1.0);
    } else if (state == MotorState::k_outtake) {
        m_intakeLeft.Set(1.0);
        m_intakeRight.Set(-1.0);
    } else if (state == MotorState::k_idle) {
        m_intakeLeft.Set(0.0);
        m_intakeRight.Set(0.0);
    }
}

void Intake::HandleEvent(Event event) {
    if (event.type == EventType::kClimberSetup) {
        m_intakeArm.Set(false);
    }
}
