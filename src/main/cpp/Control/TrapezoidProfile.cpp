// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "Control/TrapezoidProfile.hpp"

#include <cmath>

TrapezoidProfile::TrapezoidProfile(double maxV, double timeToMaxV) {
    SetMaxVelocity(maxV);
    SetTimeToMaxV(timeToMaxV);
    m_timer.Start();
}

void TrapezoidProfile::SetGoal(double goal, double currentSource) {
    std::lock_guard<std::mutex> lock(m_mutex);

    // Subtract current source for profile calculations
    m_goal = goal - currentSource;

    // Set reference to current distance since reference hasn't moved yet
    m_ref = std::make_tuple(currentSource, 0.0, 0.0);

    if (m_goal < 0.0) {
        m_sign = -1.0;
    } else {
        m_sign = 1.0;
    }
    m_timeToMaxVelocity = m_velocity / m_acceleration;

    /* d is distance traveled when accelerating to/from max velocity
     *       = 1/2 * (v0 + v) * t
     * t is m_timeToMaxVelocity
     * delta is distance traveled at max velocity
     * delta = totalDist - 2 * d
     *       = reference - 2 * ((v0 + v)/2 * t)
     * v0 = 0 therefore:
     * delta = reference - 2 * (v/2 * t)
     *       = reference - v * t
     *       = m_reference - m_velocity * m_timeToMaxVelocity
     *
     * t is time at maximum velocity
     * t = delta (from previous comment) / m_velocity (where m_velocity is
     * maximum velocity) = (m_reference - m_velocity * m_timeToMaxVelocity) /
     * m_velocity = m_reference/m_velocity - m_timeToMaxVelocity
     */
    double timeAtMaxV = m_sign * m_goal / m_velocity - m_timeToMaxVelocity;

    /* If distance travelled before reaching maximum speed is more than half of
     * the total distance to travel
     *
     * if (1/2 * a * t^2 > m_reference / 2)
     * if (a * t^2 > m_reference)
     * if (a * (v / a)^2 > m_reference)
     * if (a * v^2 / a^2 > m_reference)
     * if (v^2 / a > m_reference)
     * if (v * v / a > m_reference)
     * if (v * m_timeToMaxVelocity > m_reference)
     */
    if (m_velocity * m_timeToMaxVelocity > m_sign * m_goal) {
        /* Solve for t:
         * 1/2 * a * t^2 = m_reference / 2
         * a * t^2 = m_reference
         * t^2 = m_reference / a
         * t = std::sqrt(m_reference / a)
         */
        m_timeToMaxVelocity = std::sqrt(m_sign * m_goal / m_acceleration);
        m_timeFromMaxVelocity = m_timeToMaxVelocity;
        m_timeTotal = 2 * m_timeToMaxVelocity;
        m_profileMaxVelocity = m_acceleration * m_timeToMaxVelocity;
    } else {
        m_timeFromMaxVelocity = m_timeToMaxVelocity + timeAtMaxV;
        m_timeTotal = m_timeFromMaxVelocity + m_timeToMaxVelocity;
        m_profileMaxVelocity = m_velocity;
    }

    // Restore desired goal
    m_goal = goal;

    Reset();
}

double TrapezoidProfile::GetGoal() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_goal;
}

bool TrapezoidProfile::AtGoal() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_timer.Get() >= m_timeTotal ||
           std::abs(m_goal - std::get<0>(m_ref)) < 0.001;
}

double TrapezoidProfile::ProfileTimeTotal() const { return m_timeTotal; }

void TrapezoidProfile::SetMaxVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_velocity = velocity;
}

double TrapezoidProfile::GetMaxVelocity() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_velocity;
}

void TrapezoidProfile::SetTimeToMaxV(double timeToMaxV) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_acceleration = m_velocity / timeToMaxV;
}

double TrapezoidProfile::GetPosition() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ref = UpdateReference(m_timer.Get());
    return std::get<0>(m_ref) * m_sign;
}

double TrapezoidProfile::GetVelocity() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ref = UpdateReference(m_timer.Get());
    return std::get<1>(m_ref) * m_sign;
}

double TrapezoidProfile::GetAcceleration() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ref = UpdateReference(m_timer.Get());
    return std::get<2>(m_ref) * m_sign;
}

void TrapezoidProfile::Reset() {
    m_lastTime = 0.0;
    m_timer.Reset();
}

TrapezoidProfile::State TrapezoidProfile::UpdateReference(double currentTime) {
    if (currentTime < m_timeToMaxVelocity) {
        // Accelerate up
        std::get<2>(m_ref) = m_acceleration;
        std::get<1>(m_ref) = std::get<2>(m_ref) * currentTime;
    } else if (currentTime < m_timeFromMaxVelocity) {
        // Maintain max velocity
        std::get<2>(m_ref) = 0.0;
        std::get<1>(m_ref) = m_profileMaxVelocity;
    } else if (currentTime < m_timeTotal) {
        // Accelerate down
        double decelTime = currentTime - m_timeFromMaxVelocity;
        std::get<2>(m_ref) = -m_acceleration;
        std::get<1>(m_ref) =
            m_profileMaxVelocity + std::get<2>(m_ref) * decelTime;
    } else {
        std::get<2>(m_ref) = 0.0;
        std::get<1>(m_ref) = 0.0;
    }

    if (currentTime < m_timeTotal) {
        std::get<0>(m_ref) +=
            m_sign * std::get<1>(m_ref) * (currentTime - m_lastTime);
        m_lastTime = currentTime;
    }

    return m_ref;
}
