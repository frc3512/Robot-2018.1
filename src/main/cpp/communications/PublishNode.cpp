// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "communications/PublishNode.hpp"

#include <wpi/SmallVector.h>

#include <cstring>
#include <iostream>

PublishNode::PublishNode(std::string nodeName) {
    m_nodeName = nodeName;
    m_thread = std::thread(&PublishNode::RunFramework, this);
}

PublishNode::~PublishNode() {
    m_isRunning = false;
    m_thread.join();
}

void PublishNode::Subscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it == publisher.m_subList.end()) {
        publisher.m_subList.push_back(this);
    }
}

void PublishNode::Unsubscribe(PublishNode& publisher) {
    auto it =
        std::find(publisher.m_subList.begin(), publisher.m_subList.end(), this);
    if (it != publisher.m_subList.end()) {
        publisher.m_subList.erase(it);
    }
}

void PublishNode::ProcessMessage(const StatePacket& message) {}

void PublishNode::ProcessMessage(const ButtonPacket& message) {}

void PublishNode::RunFramework() {
    while (m_isRunning) {
        std::unique_lock<std::mutex> lock(m_mutex);

        // Waits for queue to contain messages
        m_ready.wait(lock,
                     [this] { return m_queue.size() > 0 || !m_isRunning; });

        // Pops first element out of queue for length of one whole message, then
        // pops that amount elements
        size_t msgLength = m_queue.pop_front();
        wpi::SmallVector<char, 32> message;
        for (int i = 0; i < msgLength; i++) {
            message.push_back(m_queue.pop_front());
        }

        // Checks the first byte of the message for its ID to determine which
        // packet to deserialize to, then processes it
        auto packetType = static_cast<PacketType>(message[0]);
        if (packetType == PacketType::kState) {
            StatePacket packet{message.data(), message.size()};
            m_mutex.unlock();
            ProcessMessage(packet);
            m_mutex.lock();
        } else if (packetType == PacketType::kButton) {
            ButtonPacket packet{message.data(), message.size()};
            m_mutex.unlock();
            ProcessMessage(packet);
            m_mutex.lock();
        }
    }
}
