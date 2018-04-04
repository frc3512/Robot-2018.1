// Copyright (c) 2014-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include "Logging/LogSinkBase.hpp"

/**
 * A sink for writing logged events to standard output.
 */
class LogConsoleSink : public LogSinkBase {
public:
    virtual ~LogConsoleSink() = default;

    /**
     * Write an event to standard output.
     *
     * @param event The event to log.
     */
    void Log(LogEvent event) override;
};
