// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include "Robot.hpp"

Intake::Intake() : PublishNode("Intake") {}

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

void Intake::AutoOuttake() {
    // IntakeLeft is Inverted, IntakeRight is not
    m_intakeLeft.Set(0.75);
    m_intakeRight.Set(-0.75);
}

void Intake::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 3 &&
        message.pressed) {
        if (IsOpen()) {
            Close();
        } else if (IsDeployed()) {
            Open();
        }
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 5 &&
        message.pressed && !Robot::elevator.GetBottomHallEffect()) {
        if (IsDeployed()) {
            Stow();
            Close();
        } else {
            Deploy();
        }
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
        message.pressed) {
        SetMotors(MotorState::kIntake);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
        message.pressed) {
        SetMotors(MotorState::kOuttake);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
            !message.pressed ||
        message.topic == "Robot/AppendageStick" && message.button == 6 &&
            !message.pressed) {
        SetMotors(MotorState::kIdle);
    }
}
