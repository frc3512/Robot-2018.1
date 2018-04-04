// Copyright (c) 2014-2018 FRC Team 3512. All Rights Reserved.

#include "Logging/LogConsoleSink.hpp"

#include <iostream>

void LogConsoleSink::Log(LogEvent event) {
    std::cout << event.ToFormattedString();
}
