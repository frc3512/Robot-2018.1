// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

template <class P>
void PublishNode::Publish(P p) {
    std::lock_guard<std::mutex> lock(m_mutex);
    p.topic = m_nodeName + "/" + p.topic;
    Packet packet = p.Serialize();
    size_t len = packet.getDataSize();
    auto ptr = static_cast<const char*>(packet.getData());
    for (int i = 0; i < m_subList.size(); i++) {
        m_subList[i]->m_queue.push_back(len);
        for (int j = 0; j < len; j++) {
            m_subList[i]->m_queue.push_back(ptr[j]);
        }
        m_subList[i]->m_ready.notify_one();
    }
}
