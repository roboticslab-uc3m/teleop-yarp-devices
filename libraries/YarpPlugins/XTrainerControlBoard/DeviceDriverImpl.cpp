// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool XTrainerControlBoard::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(XCB) << "Could not parse parameters";
        return false;
    }

    if (m_axes <= 0)
    {
        yCError(XCB) << "Invalid number of axes:" << m_axes;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool XTrainerControlBoard::close()
{
    return true;
}

// -----------------------------------------------------------------------------
