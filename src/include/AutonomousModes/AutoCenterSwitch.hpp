// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Timer.h>

#include "ES/Service.hpp"

class AutoCenterSwitch : public Service {
public:
    AutoCenterSwitch();

    void Reset();

    void HandleEvent(Event event) override;

private:
    frc::Timer autoTimer;

    enum class State {
        kInit,
        kInitialForward,
        kInitialRotate,
        kSecondForward,
        kFinalRotate,
        kFinalForward,
        kIdle
    };

    State state;
};
