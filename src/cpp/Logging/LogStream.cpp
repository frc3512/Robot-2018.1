// Copyright (c) 2014-2018 FRC Team 3512. All Rights Reserved.

#include "Logging/LogStream.hpp"

#include "Logging/LogStreambuf.hpp"

LogStream::LogStream(Logger& logger)
    : std::ostream(new LogStreambuf(logger)), m_logger(logger) {}

LogStream::~LogStream() { delete rdbuf(); }

void LogStream::SetLevel(LogEvent::VerbosityLevel level) {
    static_cast<LogStreambuf*>(rdbuf())->SetLevel(level);
}
