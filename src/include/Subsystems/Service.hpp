// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include "Event.hpp"

class Service {
public:
    virtual void HandleEvent(Event event) = 0;
};
