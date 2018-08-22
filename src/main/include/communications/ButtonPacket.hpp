// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/StringRef.h>

#include <string>

#include "PacketType.hpp"
#include "dsdisplay/Packet.hpp"

class ButtonPacket {
public:
    int8_t ID = static_cast<int8_t>(PacketType::kButton);
    std::string topic;
    int32_t button = 0;
    bool pressed = true;

    ButtonPacket() = default;

    /**
     * Construct a ButtonPacket with the given fields.
     *
     * @param topic The topic string of this packet.
     * @param state The integer representing the button pressed/released
     * @param pressed The boolean indicating the button was pressed if true or
     * released if false
     */
    ButtonPacket(std::string topic, int button, bool pressed);

    /**
     * Deserializes the given packet.
     *
     * @param packet The packet to deserialize.
     */
    explicit ButtonPacket(Packet& packet);

    /**
     * Deserializes the given packet.
     *
     * @param packet The buffer containing the packet.
     * @param size   The length of the packet.
     */
    ButtonPacket(const char* buf, size_t length);

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
