// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

// ------------------ IEncoders Related -----------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAxes(std::size_t & axes)
{
    axes = NUM_AXES;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::resetEncoder(int j)
{
    CHECK_JOINT(j);
    return setEncoder(j, 0.0);
  }

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::resetEncoders()
{
    bool ok = true;

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= resetEncoder(i);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setEncoder(int j, double val)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setEncoders(const double * vals)
{
    bool ok = true;

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= setEncoder(i, vals[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoder(int j, double * v)
{
    CHECK_JOINT(j);

    std::lock_guard lock(mtx);

    if (auto result = motors[j]->getPresentPosition(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get encoder value:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *v = result.value() * 360.0 / m_encoderPulses;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoders(double * encs)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < NUM_AXES; i++)
    {
        executor->addCmd(motors[i]->stageGetPresentPosition());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get encoders:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        for (auto i = 0; i < NUM_AXES; i++)
        {
            encs[i] = result.value()[i].value() * 360.0 / m_encoderPulses;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderSpeed(int j, double * sp)
{
    CHECK_JOINT(j);

    std::lock_guard lock(mtx);

    if (auto result = motors[j]->getPresentVelocity(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get encoder speed:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *sp = result.value() * m_velocityRatio;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderSpeeds(double * spds)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < NUM_AXES; i++)
    {
        executor->addCmd(motors[i]->stageGetPresentVelocity());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get encoder speeds:" << dynamixel::getErrorMessage(result.error());
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

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderAcceleration(int j, double * spds)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderAccelerations(double * accs)
{
    bool ok = true;

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= getEncoderAcceleration(i, &accs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// ------------------ IEncodersTimed Related -----------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderTimed(int j, double * encs, double * time)
{
    auto ret = getEncoder(j, encs);
    *time = yarp::os::SystemClock::nowSystem();
    return ret;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncodersTimed(double * encs, double * time)
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
