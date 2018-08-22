// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>
#include <mutex>
#include <tuple>

#include <Timer.h>

/**
 * Provides trapezoidal velocity control.
 *
 * Constant acceleration until target (max) velocity is reached, sets
 * acceleration to zero for a calculated time, then decelerates at a constant
 * acceleration with a slope equal to the negative slope of the initial
 * acceleration.
 */
class TrapezoidProfile {
public:
    TrapezoidProfile(double maxV, double timeToMaxV);
    virtual ~TrapezoidProfile() = default;

    /**
     * Sets goal state of profile.
     *
     * @param goal a distance to which to travel
     * @param currentSource the current position
     */
    void SetGoal(double goal, double currentSource = 0.0);

    /**
     * Returns profile's goal state.
     */
    double GetGoal() const;

    /**
     * Returns true if motion profile has reached goal state.
     */
    bool AtGoal() const;

    double ProfileTimeTotal() const;

    /**
     * Sets maximum velocity of profile.
     */
    void SetMaxVelocity(double velocity);

    /**
     * Returns maximum velocity of profile.
     */
    double GetMaxVelocity() const;

    /**
     * Sets time to max velocity of profile from rest.
     */
    void SetTimeToMaxV(double timeToMaxV);

    /**
     * Returns profile's current position.
     */
    double GetPosition();

    /**
     * Returns profile's current velocity.
     */
    double GetVelocity();

    /**
     * Returns profile's current acceleration.
     */
    double GetAcceleration();

    void Reset();

private:
    double m_acceleration;
    double m_velocity;
    double m_profileMaxVelocity;
    double m_timeFromMaxVelocity;
    double m_timeToMaxVelocity;

    using State = std::tuple<double, double, double>;

    State UpdateReference(double currentTime);

    // Use this to make UpdateSetpoint() and SetGoal() thread-safe
    mutable std::mutex m_mutex;

    frc::Timer m_timer;

    double m_goal = 0.0;

    double m_sign;

    // Current reference (displacement, velocity, acceleration)
    State m_ref = std::make_tuple(0.0, 0.0, 0.0);

    double m_lastTime = 0.0;
    double m_timeTotal = std::numeric_limits<double>::infinity();
};
