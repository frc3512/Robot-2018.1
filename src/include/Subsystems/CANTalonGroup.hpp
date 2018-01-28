// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <CtrlSys/INode.h>
#include <DigitalInput.h>
#include <SpeedController.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>

/**
 * Represents a gear box with up to 3 motors and an encoder
 *
 * Notes:
 * - This class uses only CANTalons.
 * - Up to three motors can be specified per gearbox, since drive train
 *   gearboxes will use up to three and other gearboxes will use less.
 */
class CANTalonGroup : public frc::SpeedController, public frc::INode {
public:
    using FeedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice;
    using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;

    template <class... CANTalons>
    CANTalonGroup(TalonSRX& canTalon, CANTalons&... canTalons);
    template <class... CANTalons>
    CANTalonGroup(int forwardLimitPin, int reverseLimitPin, TalonSRX& canTalon,
                  CANTalons&... canTalons);

    // SpeedController interface
    void Set(double value) override;
    double Get() const override;
    void SetInverted(bool isInverted) override;
    bool GetInverted() const override;
    void Disable() override;
    void StopMotor() override;
    void PIDWrite(double output) override;

    // Returns current position of master CANTalon
    double GetPosition() const;

    // Returns current speed of master CANTalon
    double GetSpeed() const;

    void SetDistancePerPulse(double distancePerPulse);

    void SetFeedbackDevice(FeedbackDevice device);

    // Resets encoder distance to 0
    void ResetEncoder();

    // Reverses gearbox encoder direction
    void SetSensorDirection(bool reverse);

    // Returns motor reversal state of gearbox
    bool IsEncoderReversed() const;

    // If true, motor is stopped when either limit switch reads high
    void SetLimitOnHigh(bool limitOnHigh);

    // Keeps gearbox within a certain position range
    void SetSoftPositionLimits(double min, double max);

    // Returns reference to master CANTalon
    TalonSRX& GetMaster();

    // INode interface
    double GetOutput() override;

private:
    bool m_isEncoderReversed = false;
    bool m_limitOnHigh = true;

    double m_min = std::numeric_limits<double>::min();
    double m_max = std::numeric_limits<double>::max();

    // Conversion factor for setpoints with respect to encoder readings
    double m_distancePerPulse = 1.0;

    // Feedback device
    FeedbackDevice m_feedbackDevice = FeedbackDevice::None;

    // Prevents motor from rotating forward when switch is pressed
    std::unique_ptr<frc::DigitalInput> m_forwardLimit;

    // Prevents motor from rotating in reverse when switch is pressed
    std::unique_ptr<frc::DigitalInput> m_reverseLimit;

    std::vector<std::reference_wrapper<TalonSRX>> m_canTalons;
};

#include "CANTalonGroup.inc"
