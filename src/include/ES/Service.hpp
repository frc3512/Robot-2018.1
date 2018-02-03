// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "Constants.hpp"
#include "Event.hpp"
#include "circular_buffer.hpp"

class Service {
public:
    Service();
    virtual ~Service();

    virtual void HandleEvent(Event event) = 0;

    void PostEvent(Event event);

private:
    circular_buffer<Event, kEventQueueSize> m_eventQueue;
    std::condition_variable m_ready;
    std::mutex m_mutex;
    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};

    void RunFramework();
};
