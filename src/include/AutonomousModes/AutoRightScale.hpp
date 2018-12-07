// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Timer.h>

#include "ES/Service.hpp"

class AutoRightScale : public Service {
public:
    AutoRightScale();

    void Reset();

    void HandleEvent(Event event) override;

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
