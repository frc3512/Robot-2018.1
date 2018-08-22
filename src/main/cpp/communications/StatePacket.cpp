// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "communications/StatePacket.hpp"

StatePacket::StatePacket(std::string topic, double state) {
    this->topic = topic;
    this->state = state;
}

StatePacket::StatePacket(Packet& packet) { Deserialize(packet); }

StatePacket::StatePacket(const char* buf, size_t length) {
    Deserialize(buf, length);
}

Packet StatePacket::Serialize() const {
    Packet packet;
    packet << ID;
    packet << topic;
    packet << state;
    return packet;
}

void StatePacket::Deserialize(Packet& packet) {
    packet >> ID;
    packet >> topic;
    packet >> state;
}

void StatePacket::Deserialize(const char* buf, size_t length) {
    Packet packet;
    packet.append(buf, length);
    Deserialize(packet);
}
