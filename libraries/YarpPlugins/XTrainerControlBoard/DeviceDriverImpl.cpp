// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <algorithm> // std::find
#include <unordered_set>

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

    if (m_motorIds.size() != NUM_AXES)
    {
        yCError(XCB) << "Invalid number of motor IDs, expected" << NUM_AXES << "but got" << m_motorIds.size();
        return false;
    }

    if (std::unordered_set(m_motorIds.begin(), m_motorIds.end()).size() != m_motorIds.size())
    {
        yCError(XCB) << "Duplicate motor IDs found in the list";
        return false;
    }

    for (const auto & id : m_motorIds)
    {
        if (id <= 0)
        {
            yCError(XCB) << "Invalid motor ID in the list:" << id;
            return false;
        }
    }

    if (m_extraId < 0)
    {
        yCError(XCB) << "Invalid extra motor ID:" << m_extraId;
        return false;
    }
    else if (m_extraId != 0 && std::find(m_motorIds.begin(), m_motorIds.end(), m_extraId) != m_motorIds.end())
    {
        yCError(XCB) << "Extra motor ID" << m_extraId << "is already in the list of motor IDs";
        return false;
    }

    if (m_encoderPulses <= 0)
    {
        yCError(XCB) << "Invalid number of encoder pulses:" << m_encoderPulses;
        return false;
    }

    if (m_velocityRatio <= 0.0)
    {
        yCError(XCB) << "Invalid velocity ratio:" << m_velocityRatio;
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

    for (const auto & id : m_motorIds)
    {
        motors.push_back(connector->createMotor(id));
    }

    if (m_extraId != 0)
    {
        yCInfo(XCB) << "Using extra motor with ID" << m_extraId;
        motors.push_back(connector->createMotor(m_extraId)); // size: NUM_AXES + 1
        motors.back()->setDirection(dynamixel::Direction::REVERSE);
    }

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
