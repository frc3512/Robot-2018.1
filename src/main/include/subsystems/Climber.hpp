// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>

#include "../Constants.hpp"
#include "Elevator.hpp"
#include "Intake.hpp"
#include "es/Service.hpp"

class Climber : public Service {
public:
    void EngagePawl();
    void LockPawl();
    void Shift();
    void HandleEvent(Event event) override;

private:
    frc::Solenoid m_pawl{kPawlPort};
    frc::DoubleSolenoid m_setupSolenoid{kSetupForwardPort, kSetupReversePort};

    frc::Timer timer;
};
