// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <DoubleSolenoid.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "ES/Service.hpp"
#include "Elevator.hpp"
#include "Intake.hpp"

class Climber : public Service {
public:
    void HandleEvent(Event event) override;

private:
    frc::DoubleSolenoid m_setupSolenoid{kSetupForwardPort, kSetupReversePort};
    frc::Solenoid m_alignmentArms{kAlignmentArmsPort};

    Timer timer;
};
