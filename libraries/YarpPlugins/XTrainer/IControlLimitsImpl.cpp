// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IControlLimits Related ------------------------------------

yarp::dev::ReturnValue XTrainer::setPosLimits(int axis, double min, double max)
{
    CHECK_JOINT(axis);

    std::lock_guard lock(mtx);

    if (auto result = motors[axis]->setMinPositionLimit(min * m_encoderPulses / 360.0 + m_offsets[axis]); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to set minimum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCIInfo(XTR, id()) << "Minimum position limit set to" << min;

    if (auto result = motors[axis]->setMaxPositionLimit(max * m_encoderPulses / 360.0 + m_offsets[axis]); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to set maximum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCIInfo(XTR, id()) << "Maximum position limit set to" << max;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPosLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);

    std::lock_guard lock(mtx);

    if (auto result = motors[axis]->getMinPositionLimit(); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to get minimum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *min = result.value() * 360.0 / m_encoderPulses - m_offsets[axis];
    }

    if (auto result = motors[axis]->getMaxPositionLimit(); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to get maximum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *max = result.value() * 360.0 / m_encoderPulses - m_offsets[axis];
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setVelLimits(int axis, double min, double max)
{
    CHECK_JOINT(axis);

    std::lock_guard lock(mtx);

    if (auto result = motors[axis]->setVelocityLimit(max / m_velocityRatio); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to set velocity limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCIInfo(XTR, id()) << "Velocity limit set to" << max;

    // assuming min = -max

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getVelLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);

    std::lock_guard lock(mtx);

    if (auto result = motors[axis]->getVelocityLimit(); !result.isSuccess())
    {
        yCIError(XTR, id()) << "Failed to get velocity limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *min = -(result.value() * m_velocityRatio);
        *max = result.value() * m_velocityRatio;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
