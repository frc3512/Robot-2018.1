// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

class AutoLeftScale {
public:
    AutoLeftScale();

    void Reset();

    void Run();

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
