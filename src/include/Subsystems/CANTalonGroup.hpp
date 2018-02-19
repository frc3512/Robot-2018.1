// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <CtrlSys/INode.h>
#include <DigitalInput.h>
#include <SpeedController.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

/**
 * Represents a gear box with an arbitrary number of motors and an encoder.
 */
class CANTalonGroup : public frc::SpeedController, public frc::INode {
public:
    using ControlMode = ctre::phoenix::motorcontrol::ControlMode;
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    template <class... CANTalons>
    CANTalonGroup(WPI_TalonSRX& canTalon, CANTalons&... canTalons);

    // SpeedController interface
    void Set(double value) override;
    double Get() const override;
    void SetInverted(bool isInverted) override;
    bool GetInverted() const override;
    void Disable() override;
    void StopMotor() override;
    void PIDWrite(double output) override;

    /**
     * Uses limit switches to create boundaries for the motors
     *
     * Passing a nullptr as a limit switch causes it not to be used.
     */
    void EnableHardLimits(frc::DigitalInput* forwardLimitSwitch,
                          frc::DigitalInput* reverseLimitSwitch);

    void SetHardLimitPressedState(bool high);

    /**
     * Uses encoders to create boundaries for the motors
     *
     * Passing positive infinity as the forward limit or negative infinity as
     * the reverse limit will cause the limit to do nothing.
     */
    void EnableSoftLimits(double forwardLimit, double reverseLimit);

    // Returns current position of master CANTalon
    double GetPosition() const;

    // Returns current speed of master CANTalon
    double GetSpeed() const;

    void SetDistancePerPulse(double distancePerPulse);

    // Resets encoder distance to 0
    void ResetEncoder();

    // Reverses gearbox encoder direction
    void SetSensorDirection(bool reverse);

    // INode interface
    double GetOutput() override;

private:
    // Conversion factor for setpoints with respect to encoder readings
    double m_distancePerPulse = 1.0;

    // Prevents motor from rotating forward when switch is pressed
    frc::DigitalInput* m_forwardLimitSwitch;

    // Prevents motor from rotating in reverse when switch is pressed
    frc::DigitalInput* m_reverseLimitSwitch;

    // Prevents motor from rotating past these encoder values
    double m_forwardLimit = std::numeric_limits<double>::infinity();
    double m_reverseLimit = -std::numeric_limits<double>::infinity();

    bool m_limitPressedState = true;

    std::vector<std::reference_wrapper<WPI_TalonSRX>> m_canTalons;
};

#include "CANTalonGroup.inc"
