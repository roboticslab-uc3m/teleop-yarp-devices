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

    if (m_port.empty())
    {
        yCError(XCB) << "Invalid empty serial port";
        return false;
    }

    if (m_baudrate <= 0)
    {
        yCError(XCB) << "Invalid baudrate:" << m_baudrate;
        return false;
    }

    if (m_ids.empty())
    {
        yCError(XCB) << "Invalid empty motor IDs list";
        return false;
    }

    for (const auto & id : m_ids)
    {
        if (id <= 0)
        {
            yCError(XCB) << "Invalid motor ID in the list:" << id;
            return false;
        }
    }

    if (m_extra_id <= 0)
    {
        yCError(XCB) << "Invalid extra motor ID:" << m_extra_id;
        return false;
    }

    connector = std::make_unique<dynamixel::Connector>(m_port, m_baudrate);

    const auto result = connector->broadcastPing();

    if (result.isSuccess())
    {
        yCInfo(XCB) << "Broadcast ping successful, got IDs:" << result.value();
    }
    else
    {
        yCError(XCB) << "Broadcast ping failed:" << dynamixel::getErrorMessage(result.error());
        return false;
    }

    for (const auto & id : m_ids)
    {
        motors.push_back(connector->createMotor(id));
    }

    motors.push_back(connector->createMotor(m_extra_id));

    for (const auto & motor : motors)
    {
        auto id = motor->getID();
        auto modelNumber = motor->getModelNumber();
        auto modelName = motor->getModelName();

        yCInfo(XCB) << "Got motor ID" << id << "with model number" << modelNumber << "and model name" << modelName;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool XTrainerControlBoard::close()
{
    if (connector)
    {
        connector->closePort();
    }

    return true;
}

// -----------------------------------------------------------------------------
