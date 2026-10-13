// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <algorithm> // std::find
#include <unordered_set>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool XTrainer::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(XTR) << "Could not parse parameters";
        return false;
    }

    if (m_port.empty())
    {
        yCError(XTR) << "Invalid empty serial port";
        return false;
    }

    if (m_baudrate <= 0)
    {
        yCError(XTR) << "Invalid baudrate:" << m_baudrate;
        return false;
    }

    if (m_motorIds.size() != NUM_MOTORS)
    {
        yCError(XTR) << "Invalid number of motor IDs, expected" << NUM_MOTORS << "but got" << m_motorIds.size();
        return false;
    }

    if (std::unordered_set(m_motorIds.begin(), m_motorIds.end()).size() != m_motorIds.size())
    {
        yCError(XTR) << "Duplicate motor IDs found in the list";
        return false;
    }

    for (const auto & id : m_motorIds)
    {
        if (id <= 0)
        {
            yCError(XTR) << "Invalid motor ID in the list:" << id;
            return false;
        }
    }

    if (m_extraId < 0)
    {
        yCError(XTR) << "Invalid extra motor ID:" << m_extraId;
        return false;
    }
    else if (m_extraId != 0 && std::find(m_motorIds.begin(), m_motorIds.end(), m_extraId) != m_motorIds.end())
    {
        yCError(XTR) << "Extra motor ID" << m_extraId << "is already in the list of motor IDs";
        return false;
    }

    if (m_encoderPulses <= 0)
    {
        yCError(XTR) << "Invalid number of encoder pulses:" << m_encoderPulses;
        return false;
    }

    if (m_velocityRatio <= 0.0)
    {
        yCError(XTR) << "Invalid velocity ratio:" << m_velocityRatio;
        return false;
    }

    if (m_offsets.size() != NUM_MOTORS)
    {
        yCError(XTR) << "Invalid number of offsets, expected" << NUM_MOTORS << "but got" << m_offsets.size();
        return false;
    }

    if (m_period <= 0.0)
    {
        yCError(XTR) << "Invalid period:" << m_period;
        return false;
    }

    connector = std::make_unique<dynamixel::Connector>(m_port, m_baudrate);

    if (auto result = connector->broadcastPing(); result.isSuccess())
    {
        yCInfo(XTR) << "Broadcast ping successful, got IDs:" << result.value();
    }
    else
    {
        yCError(XTR) << "Broadcast ping failed:" << dynamixel::getErrorMessage(result.error());
        return false;
    }

    for (const auto & id : m_motorIds)
    {
        motors.push_back(connector->createMotor(id));
    }

    if (m_extraId != 0)
    {
        yCInfo(XTR) << "Using extra motor with ID" << m_extraId;
        motors.push_back(connector->createMotor(m_extraId)); // new size: NUM_MOTORS + 1

        if (m_extraReverse)
        {
            motors.back()->setDirection(dynamixel::Direction::REVERSE);
        }
    }

    for (const auto & motor : motors)
    {
        auto id = motor->getID();
        auto modelNumber = motor->getModelNumber();
        auto modelName = motor->getModelName();

        yCInfo(XTR) << "Got motor ID" << id << "with model number" << modelNumber << "and model name" << modelName;
    }

    lastJointPositions.resize(NUM_MOTORS);
    lastButtonStates.resize(NUM_BUTTONS);

    if (!yarp::os::PeriodicThread::setPeriod(m_period) || !yarp::os::PeriodicThread::start())
    {
        yCError(XTR) << "Failed to start periodic thread with period:" << m_period;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool XTrainer::close()
{
    if (yarp::os::PeriodicThread::isRunning())
    {
        yarp::os::PeriodicThread::stop();
    }

    if (connector)
    {
        connector->closePort();
        connector.reset();
    }

    return true;
}

// -----------------------------------------------------------------------------
