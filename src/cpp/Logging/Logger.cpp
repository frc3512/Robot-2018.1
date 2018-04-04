// Copyright (c) 2014-2018 FRC Team 3512. All Rights Reserved.

#include "Logging/Logger.hpp"

#include <algorithm>

Logger::Logger() { ResetInitialTime(); }

void Logger::Log(LogEvent event) {
    event.SetInitialTime(m_initialTime);

    for (auto sink : m_sinkList) {
        if (sink.get().TestVerbosityLevel(event.GetVerbosityLevel())) {
            sink.get().Log(event);
        }
    }
}

void Logger::AddLogSink(LogSinkBase& sink) { m_sinkList.emplace_back(sink); }

void Logger::RemoveLogSink(LogSinkBase& sink) {
    m_sinkList.erase(
        std::remove_if(m_sinkList.begin(), m_sinkList.end(),
                       [&](std::reference_wrapper<LogSinkBase> elem) -> bool {
                           return elem.get() == sink;
                       }),
        m_sinkList.end());
}

Logger::LogSinkBaseList Logger::ListLogSinks() const { return m_sinkList; }

void Logger::ResetInitialTime() { m_initialTime = std::time(nullptr); }

void Logger::SetInitialTime(std::time_t time) { m_initialTime = time; }
