// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "control/Pathfinder.hpp"

#include "Constants.hpp"

TrajectoryPoint Pathfinder::GeneratePath() {
    using wheel_state_t = grpl::pf::coupled::wheel_state;
    using grpl::pf::VELOCITY;

    m_t += kDt;
    m_state = m_gen.generate(m_chassis, m_curves.begin(), m_curves.end(),
                             m_profile, m_state, m_t);
    std::pair<wheel_state_t, wheel_state_t> split = m_chassis.split(m_state);

    m_w = (split.second.kinematics[VELOCITY] -
           split.first
               .kinematics[VELOCITY]);  // track radius = 0.5, track diam = 1
    m_v =
        (split.second.kinematics[VELOCITY] + split.first.kinematics[VELOCITY]) /
        2.0;

    m_center[2] += m_w * kDt;
    m_center[0] += m_v * std::cos(m_center[2]) * kDt;
    m_center[1] += m_v * std::sin(m_center[2]) * kDt;

    return TrajectoryPoint(m_center[0], m_center[1], m_center[2], m_v, m_w);
}
