// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "DiffDriveController.hpp"

#include <iostream>

#include "Robot.hpp"

using namespace frc;

/**
 * Constructs DiffDriveController.
 *
 * @param positionRef position reference input
 * @param angleRef angle reference input
 * @param leftEncoder left encoder
 * @param rightEncoder right encoder
 * @param angleSensor angle sensor (e.g, gyroscope)
 * @param clockwise true if clockwise rotation increases angle measurement
 * @param leftMotor left motor output
 * @param rightMotor right motor output
 * @param period the loop time for doing calculations.
 */
DiffDriveController::DiffDriveController(
    MotionProfile& positionRef, MotionProfile& angleRef, INode& leftEncoder,
    INode& rightEncoder, INode& angleSensor, bool clockwise,
    PIDOutput& leftMotor, PIDOutput& rightMotor, double period)
    : m_positionRef(positionRef),
      m_angleRef(angleRef),
      m_leftEncoder(leftEncoder),
      m_rightEncoder(rightEncoder),
      m_angleSensor(angleSensor),
      m_clockwise(clockwise),
      m_leftMotor(leftMotor),
      m_rightMotor(rightMotor),
      m_leftMotorInput(m_positionPID, false, m_anglePID, !m_clockwise,
                       m_positionFeedForward, false, m_angleFeedForward,
                       !m_clockwise),
      m_leftOutput(m_leftMotorInput, m_leftMotor),
      m_rightMotorInput(m_positionPID, true, m_anglePID, !m_clockwise,
                        m_positionFeedForward, true, m_angleFeedForward,
                        !m_clockwise),
      m_rightOutput(m_rightMotorInput, m_rightMotor),
      m_outputs(m_leftOutput, m_rightOutput),
      m_period(period) {}

void DiffDriveController::Enable() { m_outputs.Enable(m_period); }

void DiffDriveController::Disable() { m_outputs.Disable(); }

PIDNode& DiffDriveController::GetPositionPID() { return m_positionPID; }

PIDNode& DiffDriveController::GetAnglePID() { return m_anglePID; }

double DiffDriveController::GetPosition() { return m_positionCalc.GetOutput(); }

double DiffDriveController::GetAngle() { return m_angleSensor.GetOutput(); }

void DiffDriveController::SetPositionTolerance(double tolerance,
                                               double deltaTolerance) {
    m_positionError.SetTolerance(tolerance, deltaTolerance);
}

void DiffDriveController::SetAngleTolerance(double tolerance,
                                            double deltaTolerance) {
    m_angleError.SetTolerance(tolerance, deltaTolerance);
}

bool DiffDriveController::AtPosition() const {
    return m_positionError.InTolerance() && m_positionRef.AtGoal();
}

bool DiffDriveController::AtAngle() const {
    return m_angleError.InTolerance() && m_angleRef.AtGoal();
}

void DiffDriveController::Debug() {
    Robot::liveGrapher.GraphData(m_positionRef.GetPositionNode().GetOutput(),
                                 "Position Reference");
    Robot::liveGrapher.GraphData(m_angleRef.GetPositionNode().GetOutput(),
                                 "Angle Reference");
    // Robot::liveGrapher.GraphData(m_rightEncoder.GetOutput(), "Right
    // Encoder");  Robot::liveGrapher.GraphData(m_leftEncoder.GetOutput(), "Left
    // Encoder");
    Robot::liveGrapher.GraphData(
        (m_leftEncoder.GetOutput() + m_rightEncoder.GetOutput()) / 2,
        "Encoder Average");
    Robot::liveGrapher.GraphData(m_leftMotorInput.GetOutput(),
                                 "Left Motor Input");
    Robot::liveGrapher.GraphData(m_rightMotorInput.GetOutput(),
                                 "Right Motor Input");
    // Robot::liveGrapher.GraphData(m_positionError.GetOutput(), "Position
    // Error");
    // Robot::liveGrapher.GraphData(m_angleError.GetOutput(), "Angle Error");
    // std::cout << "InTolerance: " << m_positionError.InTolerance()
    //          << " AtGoal: " << m_positionRef.AtGoal() << std::endl;
    Robot::liveGrapher.GraphData(m_angleSensor.GetOutput(), "Angle");
    Robot::liveGrapher.GraphData(m_positionRef.GetVelocityNode().GetOutput(),
                                 "Velocity Reference");
    Robot::liveGrapher.GraphData(m_angleRef.GetVelocityNode().GetOutput(),
                                 "Angle Rate Reference");
    Robot::liveGrapher.GraphData(m_angleFeedForward.GetOutput(),
                                 "Angle FeedForward");
    Robot::liveGrapher.GraphData(m_angleRef.GetAccelerationNode().GetOutput(),
                                 "Angle Acceleration Reference");
}
