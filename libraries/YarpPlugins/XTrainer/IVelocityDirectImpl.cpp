// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <numeric> // std::iota

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IVelocityDirect Related --------------------------------

yarp::dev::ReturnValue XTrainer::setRefVelocity(int j, double ref)
{
    CHECK_JOINT(j);

    if (j == EXTRA_MOTOR_IDX && motors.size() > NUM_MOTORS)
    {
        std::vector indices = {j};
        std::vector values = {ref};
        return setRefVelocity(indices, values);
    }

    std::lock_guard lock(mtx);

    if (auto result = motors[j]->setGoalVelocity(ref / m_velocityRatio); !result.isSuccess())
    {
        yCError(XTR) << "Failed to set velocity:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setRefVelocity(const std::vector<double> & vels)
{
    std::vector<int> indices(NUM_AXES);
    std::iota(indices.begin(), indices.end(), 0);
    return setRefVelocity(indices, vels);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setRefVelocity(const std::vector<int> & jnts, const std::vector<double> & vels)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < jnts.size(); i++)
    {
        executor->addCmd(motors[jnts[i]]->stageSetGoalVelocity(vels[i] / m_velocityRatio));

        if (jnts[i] == EXTRA_MOTOR_IDX && motors.size() > NUM_MOTORS)
        {
            executor->addCmd(motors[NUM_MOTORS]->stageSetGoalVelocity(vels[i] / m_velocityRatio));
        }
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XTR) << "Failed to set velocities:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getRefVelocity(int joint, double & ref)
{
    CHECK_JOINT(joint);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getRefVelocity(std::vector<double> & vels)
{
    bool ok = true;

    vels.resize(NUM_AXES);

    for (auto j = 0; j < NUM_AXES; j++)
    {
        ok &= getRefVelocity(j, vels[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getRefVelocity(const std::vector<int> & jnts, std::vector<double> & vels)
{
    bool ok = true;

    vels.resize(jnts.size());

    for (auto i = 0; i < jnts.size(); i++)
    {
        ok &= getRefVelocity(jnts[i], vels[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
