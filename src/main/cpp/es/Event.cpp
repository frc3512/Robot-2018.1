// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "es/Event.hpp"

Event::Event(EventType type, int32_t param) : type(type), param(param) {}

bool Event::operator==(const Event& rhs) const {
    return type == rhs.type && param == rhs.param;
}
