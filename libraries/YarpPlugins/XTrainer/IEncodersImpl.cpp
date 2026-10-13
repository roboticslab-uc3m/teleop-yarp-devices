// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <cmath>

#include <algorithm> // std::copy

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

// ------------------ IEncoders Related -----------------------------------------

yarp::dev::ReturnValue XTrainer::getAxes(std::size_t & axes)
{
    axes = NUM_AXES;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::resetEncoder(int j)
{
    return setEncoder(j, 0.0);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::resetEncoders()
{
    std::vector vals(NUM_AXES, 0.0);
    return setEncoders(vals.data());
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setEncoder(int j, double val)
{
    CHECK_JOINT(j);
    std::lock_guard lock(mtx);
    m_offsets[j] += val;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setEncoders(const double * vals)
{
    std::lock_guard lock(mtx);

    for (auto i = 0; i < NUM_AXES; i++)
    {
        m_offsets[i] += vals[i];
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoder(int j, double * v)
{
    CHECK_JOINT(j);
    std::lock_guard lock(mtx);
    *v = lastJointPositions[j];
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoders(double * encs)
{
    std::lock_guard lock(mtx);
    std::copy(lastJointPositions.begin(), lastJointPositions.end(), encs);
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoderSpeed(int j, double * sp)
{
    CHECK_JOINT(j);

    std::lock_guard lock(mtx);

    if (auto result = motors[j]->getPresentVelocity(); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to get encoder speed:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *sp = result.value() * m_velocityRatio;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoderSpeeds(double * spds)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < NUM_AXES; i++)
    {
        executor->addCmd(motors[i]->stageGetPresentVelocity());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to get encoder speeds:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        for (auto i = 0; i < NUM_AXES; i++)
        {
            spds[i] = result.value()[i].value() * m_velocityRatio;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoderAcceleration(int j, double * spds)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoderAccelerations(double * accs)
{
    bool ok = true;

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= getEncoderAcceleration(i, &accs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// ------------------ IEncodersTimed Related -----------------------------------------

yarp::dev::ReturnValue XTrainer::getEncoderTimed(int j, double * encs, double * time)
{
    auto ret = getEncoder(j, encs);
    *time = yarp::os::SystemClock::nowSystem();
    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getEncodersTimed(double * encs, double * time)
{
    auto ret = getEncoders(encs);
    auto now = yarp::os::SystemClock::nowSystem();

    for (auto i = 0; i < NUM_AXES; i++)
    {
        time[i] = now;
    }

    return ret;
}

// -----------------------------------------------------------------------------
