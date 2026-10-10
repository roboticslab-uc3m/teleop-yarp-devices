// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IControlLimits Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPosLimits(int axis, double min, double max)
{
    CHECK_JOINT(axis);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPosLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setVelLimits(int axis, double min, double max)
{
    yCWarning(XCB) << "setVelLimits() not implemented";
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getVelLimits(int axis, double * min, double * max)
{
    CHECK_JOINT(axis);

    // yarpmotorgui's defaults (partitem.cpp)
    *min = -100.0;
    *max = 100.0;

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
