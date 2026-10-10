// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/SystemClock.h>

// ------------------ IEncoders Related -----------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAxes(std::size_t & axes)
{
    axes = m_axes;
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

    for (unsigned int i = 0; i < m_axes; i++)
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

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= setEncoder(i, vals[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoder(int j, double * v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoders(double * encs)
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoder(i, &encs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderSpeed(int j, double * sp)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderSpeeds(double * spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoderSpeed(i, &spds[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
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

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoderAcceleration(i, &accs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// ------------------ IEncodersTimed Related -----------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncodersTimed(double * encs, double * time)
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getEncoderTimed(i, &encs[i], &time[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getEncoderTimed(int j, double * encs, double * time)
{
    auto ret = getEncoder(j, encs);
    *time = yarp::os::SystemClock::nowSystem();
    return ret;
}

// -----------------------------------------------------------------------------
