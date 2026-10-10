// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPositionDirect Related --------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPosition(int j, double ref)
{
    CHECK_JOINT(j);

    if (auto result = motors[j]->setGoalPosition(ref * m_encoderPulses / 360.0); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set position:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(const double * refs)
{
    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < motors.size(); i++)
    {
        executor->addCmd(motors[i]->stageSetGoalPosition(refs[i] * m_encoderPulses / 360.0));
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set positions:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
{
    auto executor = connector->createGroupExecutor();

    for (int i = 0; i < n_joint; i++)
    {
        executor->addCmd(motors[joints[i]]->stageSetGoalPosition(refs[i] * m_encoderPulses / 360.0));
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set positions:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPosition(int joint, double * ref)
{
    CHECK_JOINT(joint);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPositions(double * refs)
{
    bool ok = true;

    for (int j = 0; j < motors.size(); j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
