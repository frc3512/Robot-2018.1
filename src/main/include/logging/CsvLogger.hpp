// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctime>
#include <fstream>
#include <string>

#include <hal/cpp/fpga_clock.h>

/*
 * Class to log values separated by commas
 */
class CsvLogger {
public:
    explicit CsvLogger(const std::string& filename);

    template <class... doubles>
    void Log(double value, doubles... values);

private:
    template <class... doubles>
    void LogImpl(double value, doubles... values);

    void LogImpl(double value);

    double Timestamp();

    std::ofstream m_logfile;
    std::time_t m_timestamp;
    hal::fpga_clock::time_point m_startTime;
};

#include "logging/CsvLogger.inc"
