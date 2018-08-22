// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#include "communications/ButtonPacket.hpp"

#include <iostream>

ButtonPacket::ButtonPacket(std::string topic, int button, bool pressed) {
    this->topic = topic;
    this->button = button;
    this->pressed = pressed;
}

ButtonPacket::ButtonPacket(Packet& packet) { Deserialize(packet); }

ButtonPacket::ButtonPacket(const char* buf, size_t length) {
    Deserialize(buf, length);
}

Packet ButtonPacket::Serialize() const {
    Packet packet;
    packet << ID;
    packet << topic;
    packet << button;
    packet << pressed;
    return packet;
}

void ButtonPacket::Deserialize(Packet& packet) {
    packet >> ID;
    packet >> topic;
    packet >> button;
    packet >> pressed;
}

void ButtonPacket::Deserialize(const char* buf, size_t length) {
    Packet packet;
    packet.append(buf, length);
    Deserialize(packet);
}
