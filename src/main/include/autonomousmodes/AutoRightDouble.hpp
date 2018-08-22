// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

class AutoRightDouble {
public:
    AutoRightDouble();

    void Reset();

    void Run();

    frc::Timer autoTimer;

    enum class State {
        kInit,
        kInitialForward,
        kRightRotate,
        kFirstForward,
        kFinalRotate,
        kFinalForward,
        kDoubleRotate,
        kDoubleForward,
        kSpit,
        kIdle
    };

    State state;
};
