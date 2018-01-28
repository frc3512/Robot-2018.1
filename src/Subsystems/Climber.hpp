// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Joystick.h>
#include <Solenoid.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "Elevator.hpp"
#include "Intake.hpp"
#include "Service.hpp"

class Climber : public Service {
public:
    void HandleEvent(Event event) override;

private:
    frc::Solenoid m_setupSolenoid{k_setupSolenoidPort};
    frc::Solenoid m_alignmentArms{k_alignmentArmsPort};

    Timer timer;
};
