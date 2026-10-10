// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IControlLimits Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPosLimits(int axis, double min, double max)
{
    CHECK_JOINT(axis);

    if (auto result = motors[axis]->setMinPositionLimit(min); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set minimum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCInfo(XCB) << "Minimum position limit set to" << min;

    if (auto result = motors[axis]->setMaxPositionLimit(max); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set maximum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCInfo(XCB) << "Maximum position limit set to" << max;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPosLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);

    if (auto result = motors[axis]->getMinPositionLimit(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get minimum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *min = result.value();
    }

    if (auto result = motors[axis]->getMaxPositionLimit(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get maximum position limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *max = result.value();
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setVelLimits(int axis, double min, double max)
{
    CHECK_JOINT(axis);

    if (auto result = motors[axis]->setVelocityLimit(max); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set velocity limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    yCInfo(XCB) << "Velocity limit set to" << max;

    // assuming min = -max

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getVelLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);

    if (auto result = motors[axis]->getVelocityLimit(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get velocity limit:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *min = -result.value();
        *max = result.value();
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
