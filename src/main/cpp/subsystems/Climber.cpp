// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State { kInit, kSetup, kWaiting, kClimb, kIdle };

void Climber::EngagePawl() { m_pawl.Set(true); }

void Climber::LockPawl() { m_pawl.Set(false); }

void Climber::Shift() {
    if (m_setupSolenoid.Get() == frc::DoubleSolenoid::kForward) {
        m_setupSolenoid.Set(frc::DoubleSolenoid::kReverse);  // Low gear
    } else {
        m_setupSolenoid.Set(frc::DoubleSolenoid::kForward);  // High gear
    }
}

void Climber::HandleEvent(Event event) {
    if (Robot::driveStick2.GetRawButton(7) &&
        event == Event{kButtonPressed, 2}) {
        Shift();
    }
    if (Robot::driveStick2.GetRawButton(10) &&
        event == Event{kButtonPressed, 10}) {
        EngagePawl();
    }
}
