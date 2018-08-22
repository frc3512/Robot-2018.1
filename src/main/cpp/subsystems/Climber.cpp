// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State { kInit, kSetup, kWaiting, kClimb, kIdle };

Climber::Climber() : PublishNode("Climber") {}

void Climber::EngagePawl() { m_pawl.Set(true); }

void Climber::LockPawl() { m_pawl.Set(false); }

void Climber::Shift() {
    if (m_setupSolenoid.Get() == frc::DoubleSolenoid::kForward) {
        m_setupSolenoid.Set(frc::DoubleSolenoid::kReverse);  // Low gear
    } else {
        m_setupSolenoid.Set(frc::DoubleSolenoid::kForward);  // High gear
    }
}

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (Robot::driveStick2.GetRawButton(7) &&
        message.topic == "Robot/AppendageStick" && message.button == 2 &&
        message.pressed == true) {
        Shift();
    }
    if (Robot::driveStick2.GetRawButton(10) &&
        message.topic == "Robot/AppendageStick" && message.button == 10 &&
        message.pressed == true) {
        EngagePawl();
    }
}
