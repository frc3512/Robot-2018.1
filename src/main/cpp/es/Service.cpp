// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "es/Service.hpp"

Service::Service() { m_thread = std::thread(&Service::RunFramework, this); }

Service::~Service() {
    m_isRunning = false;
    m_ready.notify_one();
    m_thread.join();
}

void Service::PostEvent(Event event) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_eventQueue.push_back(event);
    }
    m_ready.notify_one();
}

void Service::RunFramework() {
    while (m_isRunning) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_ready.wait(
            lock, [this] { return m_eventQueue.size() > 0 || !m_isRunning; });

        while (m_eventQueue.size() > 0) {
            Event m_event = m_eventQueue.pop_front();
            m_mutex.unlock();
            HandleEvent(m_event);
            m_mutex.lock();
        }
    }
}
