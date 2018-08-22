// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/StringRef.h>

#include <string>

#include "communications/PacketType.hpp"
#include "dsdisplay/Packet.hpp"

class StatePacket {
public:
    int8_t ID = static_cast<int8_t>(PacketType::kState);
    std::string topic;
    double state = 0.0;

    StatePacket() = default;

    /**
     * Construct a StatePacket with the given fields.
     *
     * @param topic The topic string of this packet.
     * @param state The state from a feedback controller.
     */
    StatePacket(std::string topic, double state);

    /**
     * Deserializes the given packet.
     *
     * @param packet The packet to deserialize.
     */
    explicit StatePacket(Packet& packet);

    /**
     * Deserializes the given packet.
     *
     * @param packet The buffer containing the packet.
     * @param size   The length of the packet.
     */
    StatePacket(const char* buf, size_t length);

    /**
     * Serializes the contents of this class into a Packet.
     *
     * The contents of the packet should be passed to whatever communication
     * layer that takes a raw buffer.
     */
    Packet Serialize() const;

    /**
     * Deserializes the given packet.
     *
     * @param packet The buffer containing the packet.
     * @param size   The length of the packet.
     */
    void Deserialize(const char* buf, size_t size);

    /**
     * Deserializes the given packet.
     *
     * @param packet The packet to deserialize.
     */
    void Deserialize(Packet& packet);
};
