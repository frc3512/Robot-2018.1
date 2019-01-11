// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "Control/DrivetrainController.hpp"

#include <cmath>
#include <iostream>

#include "Control/Pose.hpp"

DrivetrainController::DrivetrainController() { m_Y.setZero(); }

void DrivetrainController::Enable() { m_loop.Enable(); }

void DrivetrainController::Disable() { m_loop.Disable(); }

void DrivetrainController::SetGoal(const Pose& pose) {
    m_positionProfile.SetGoal(
        std::hypot(pose.x, pose.y),
        std::hypot(m_estimatedTrajectory.x, m_estimatedTrajectory.y));
    m_angleProfile.SetGoal(pose.theta, m_estimatedTrajectory.theta);
}

void DrivetrainController::SetGoal(Pose&& pose) {
    m_positionProfile.SetGoal(
        std::hypot(pose.x, pose.y),
        std::hypot(m_estimatedTrajectory.x, m_estimatedTrajectory.y));
    m_angleProfile.SetGoal(pose.theta, m_estimatedTrajectory.theta);
}

bool DrivetrainController::AtGoal() const {
    return m_positionProfile.AtGoal() && m_angleProfile.AtGoal();
}

void DrivetrainController::SetMeasuredStates(double leftVelocity,
                                             double rightVelocity,
                                             double heading) {
    m_Y << leftVelocity, rightVelocity;
    m_headingMeasurement = deg2rad(heading);
}

double DrivetrainController::ControllerLeftVoltage() const {
    return m_loop.U(0);
}

double DrivetrainController::ControllerRightVoltage() const {
    return m_loop.U(1);
}

double DrivetrainController::EstimatedLeftVelocity() const {
    return m_loop.Xhat(0);
}

double DrivetrainController::EstimatedRightVelocity() const {
    return m_loop.Xhat(1);
}

double DrivetrainController::LeftVelocityError() const {
    return m_loop.Error()(0, 0);
}

double DrivetrainController::RightVelocityError() const {
    return m_loop.Error()(1, 0);
}

TrajectoryPoint DrivetrainController::EstimatedPose() const {
    return m_estimatedTrajectory;
}

TrajectoryPoint DrivetrainController::GoalPose() const { return m_goal; }

double DrivetrainController::LeftVelocityReference() { return m_loop.NextR(0); }

double DrivetrainController::RightVelocityReference() {
    return m_loop.NextR(1);
}

void DrivetrainController::Update() {
    m_goal = m_pathfinder.GeneratePath();

    // Update actual pose from sensors
    double v = (EstimatedLeftVelocity() + EstimatedRightVelocity()) / 2.0;
    m_estimatedTrajectory.x += v * std::cos(m_headingMeasurement) * kDt;
    m_estimatedTrajectory.y += v * std::sin(m_headingMeasurement) * kDt;
    m_estimatedTrajectory.theta = m_headingMeasurement;

    auto refs = Ramsete(m_goal, m_estimatedTrajectory, kB, kZeta);
    auto vels = GetDiffVelocities(std::get<0>(refs), std::get<1>(refs),
                                  kWheelbaseWidth);
    SetReferences(std::get<0>(vels), std::get<1>(vels));

    m_loop.Correct(m_Y);

    auto error = m_loop.Error();
    m_atReferences = std::abs(error(0, 0)) < kVelocityTolerance &&
                     std::abs(error(1, 0)) < kVelocityTolerance;

    m_loop.Predict();
}

void DrivetrainController::Reset() {
    m_loop.Reset();
    m_estimatedTrajectory = TrajectoryPoint(0, 0, 0, 0, 0);
}

void DrivetrainController::SetReferences(double leftVelocity,
                                         double rightVelocity) {
    Eigen::Matrix<double, 2, 1> nextR;
    nextR << leftVelocity, rightVelocity;
    m_loop.SetNextR(nextR);
}

double DrivetrainController::Sinc(double x) {
    if (std::abs(x) < 1e-9) {
        return 1.0;
    } else {
        return std::sin(x) / x;
    }
}

std::tuple<double, double> DrivetrainController::Ramsete(
    TrajectoryPoint trajectory_desired, TrajectoryPoint trajectory, double b,
    double zeta) {
    TrajectoryPoint e = trajectory_desired - trajectory;
    double k = 2 * zeta *
               std::sqrt(std::pow(trajectory_desired.theta, 2) +
                         b * std::pow(trajectory_desired.v, 2));

    double v = trajectory_desired.v * std::cos(e.theta) +
               k * (std::cos(trajectory.theta) * e.x +
                    std::sin(trajectory.theta) * e.y);
    double omega = (trajectory_desired.w +
                    b * trajectory_desired.v * Sinc(e.theta) *
                        (e.y * std::cos(trajectory.theta) -
                         std::sin(trajectory.theta) * e.x) +
                    k * e.theta);
    return std::make_tuple(v, omega);
}

std::tuple<double, double> DrivetrainController::GetDiffVelocities(double v,
                                                                   double omega,
                                                                   double d) {
    return std::make_tuple(v - omega * d / 2.0, v + omega * d / 2.0);
}
