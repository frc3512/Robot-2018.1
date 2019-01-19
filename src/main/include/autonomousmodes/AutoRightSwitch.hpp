// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

#include "es/Service.hpp"

class AutoRightSwitch : public Service {
public:
    AutoRightSwitch();

    void Reset();

    void HandleEvent(Event event) override;

private:
    frc::Timer autoTimer;

    enum class State {
        kInit,
        kInitialForward,
        kRightRotate,
        kRightForward,
        kFinalRotate,
        kFinalForward,
        kIdle
    };

    State state;
};
