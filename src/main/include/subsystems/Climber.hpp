// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/Elevator.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/SubsystemBase.hpp"

class Climber : public SubsystemBase, public PublishNode {
public:
    Climber();
    void EngagePawl();
    void LockPawl();
    void Shift();
    void ProcessMessage(const ButtonPacket& message) override;

private:
    frc::Solenoid m_pawl{kPawlPort};
    frc::DoubleSolenoid m_setupSolenoid{kSetupForwardPort, kSetupReversePort};

    frc::Timer timer;
};
