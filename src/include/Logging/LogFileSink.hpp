// Copyright (c) 2014-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <fstream>
#include <string>

#include "Logging/LogSinkBase.hpp"

/**
 * A file sink for the logged events.
 */
class LogFileSink : public LogSinkBase {
public:
    explicit LogFileSink(std::string filename);
    virtual ~LogFileSink() = default;

    /**
     * Write an event to the logfile.
     *
     * @param event The event to log.
     */
    void Log(LogEvent event) override;

private:
    std::ofstream m_logfile;
};
