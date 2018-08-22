// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

class AutoRightScale {
public:
    AutoRightScale();

    void Reset();

    void Run();

private:
    frc::Timer autoTimer;
    enum class State {
        kInit,
        kInitialForward,
        kLeftRotate,
        kLeftForward,
        kFinalRotate,
        kFinalForward,
        kPrepReverse,
        kPrepRotate,
        kIdle
    };

    State state;
};
