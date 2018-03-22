// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Timer.h>

#include "ES/Service.hpp"

class AutoRightDouble : public Service {
public:
    AutoRightDouble();

    void Reset();

    void HandleEvent(Event event) override;

private:
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
