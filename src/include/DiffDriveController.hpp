// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/FuncNode.h>
#include <CtrlSys/INode.h>
#include <CtrlSys/MotionProfile.h>
#include <CtrlSys/Output.h>
#include <CtrlSys/OutputGroup.h>
#include <CtrlSys/PIDNode.h>
#include <CtrlSys/SumNode.h>
#include <RobotController.h>

#include "Constants.hpp"
#include "Controller.h"
#include "PIDOutput.h"

namespace frc {

/**
 * A feedback controller for a differential-drive robot. A differential-drive
 * robot has left and right wheels separated by an arbitrary width.
 *
 * A forward distance controller and angle controller are run in parallel and
 * their outputs are composed to drive each wheel. Since the forward controller
 * uses the average distance of the two sides while the angle controller uses
 * the difference between them, the controllers act independently on the drive
 * base and can thus be tuned separately.
 *
 * If you don't have a gyroscope for an angle sensor, the following equation can
 * be used in a FuncNode to estimate it.
 *
 * angle = (right - left) / width * 180 / pi
 *
 * where "right" is the right encoder reading, "left" is the left encoder
 * reading, "width" is the width of the robot in the same units as the
 * encoders, and "angle" is the angle of the robot in degrees. We recommend
 * passing this angle estimation through a low-pass filter (see LinearFilter).
 *
 * Set the position and angle PID constants via GetPositionPID()->SetPID() and
 * GetAnglePID()->SetPID() before enabling this controller.
 */
class DiffDriveController : public Controller {
public:
    DiffDriveController(MotionProfile& positionRef, MotionProfile& angleRef,
                        INode& leftEncoder, INode& rightEncoder,
                        INode& angleSensor, bool clockwise,
                        PIDOutput& leftMotor, PIDOutput& rightMotor,
                        double period = INode::kDefaultPeriod);
    virtual ~DiffDriveController() = default;

    void Enable() override;
    void Disable() override;

    PIDNode& GetPositionPID();
    PIDNode& GetAnglePID();

    double GetPosition();
    double GetAngle();

    void SetPositionTolerance(double tolerance, double deltaTolerance);
    void SetAngleTolerance(double tolerance, double deltaTolerance);

    bool AtPosition() const;
    bool AtAngle() const;

    void Debug();

private:
    // Control system references
    MotionProfile& m_positionRef;
    MotionProfile& m_angleRef;

    // Encoders
    INode& m_leftEncoder;
    INode& m_rightEncoder;

    // Angle sensor (e.g., gyroscope)
    INode& m_angleSensor;
    bool m_clockwise;

    // Motors
    PIDOutput& m_leftMotor;
    PIDOutput& m_rightMotor;

    FuncNode m_positionFeedForward{[&] {
        return (kVDrive * m_positionRef.GetVelocityNode().GetOutput() +
                kADrive * m_positionRef.GetAccelerationNode().GetOutput()) *
               kMaxControlVoltage / RobotController::GetInputVoltage();
    }};
    FuncNode m_angleFeedForward{[&] {
        return deg2rad(kVAngle * m_angleRef.GetVelocityNode().GetOutput() *
                           kWheelbaseWidth / 2.0 +
                       kAAngle * m_angleRef.GetAccelerationNode().GetOutput() *
                           kWheelbaseWidth / 2.0) *
               kMaxControlVoltage / RobotController::GetInputVoltage();
    }};

    // Position PID
    FuncNode m_positionCalc{[&] {
        return (m_leftEncoder.GetOutput() + m_rightEncoder.GetOutput()) / 2.0;
    }};
    SumNode m_positionError{m_positionRef.GetPositionNode(), true,
                            m_positionCalc, false};
    PIDNode m_positionPID{0.0, 0.0, 0.0, m_positionError};

    // Angle PID
    SumNode m_angleError{m_angleRef.GetPositionNode(), true, m_angleSensor,
                         false};
    PIDNode m_anglePID{0.0, 0.0, 0.0, m_angleError};

    // Combine outputs for left motor
    SumNode m_leftMotorInput;
    Output m_leftOutput;

    // Combine outputs for right motor
    SumNode m_rightMotorInput;
    Output m_rightOutput;

    OutputGroup m_outputs;
    double m_period;
};

}  // namespace frc
