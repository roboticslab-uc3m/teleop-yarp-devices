// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IVelocityDirect Related --------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setRefVelocity(int j, double ref)
{
    CHECK_JOINT(j);

    if (auto result = motors[j]->setGoalVelocity(ref); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set velocity:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setRefVelocity(const std::vector<double> & vels)
{
    auto executor = connector->createGroupExecutor();

    for (int j = 0; j < motors.size(); j++)
    {
        executor->addCmd(motors[j]->stageSetGoalVelocity(vels[j]));
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set velocities:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setRefVelocity(const std::vector<int> & jnts, const std::vector<double> & vels)
{
    auto executor = connector->createGroupExecutor();

    for (size_t i = 0; i < jnts.size(); i++)
    {
        executor->addCmd(motors[jnts[i]]->stageSetGoalVelocity(vels[i]));
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set velocities:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefVelocity(int joint, double & ref)
{
    CHECK_JOINT(joint);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefVelocity(std::vector<double> & vels)
{
    bool ok = true;

    for (int j = 0; j < motors.size(); j++)
    {
        ok &= getRefVelocity(j, vels[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefVelocity(const std::vector<int> & jnts, std::vector<double> & vels)
{
    bool ok = true;

    for (size_t i = 0; i < jnts.size(); i++)
    {
        ok &= getRefVelocity(jnts[i], vels[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
