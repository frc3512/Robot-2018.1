// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

class AutoCenterScale {
public:
    AutoCenterScale();

    void Reset();

    void Run();

private:
    frc::Timer autoTimer;

    enum class State {
        kInit,
        kInitialForward,
        kInitialRotate,
        kSecondForward,
        kSecondRotate,
        kThirdForward,
        kFinalRotate,
        kFinalForward,
        kIdle
    };

    State state;
};
