// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <algorithm> // std::find
#include <unordered_set>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool XTrainer::open(yarp::os::Searchable & config)
{
    if (yarp::dev::DeviceDriver::id() == "XTrainer")
    {
        // avoid verbose log if using default ID (i.e., if not invoked via yarprobotinterface)
        yarp::dev::DeviceDriver::setId("");
    }

    if (!parseParams(config))
    {
        yCIError(XTR, id()) << "Could not parse parameters";
        return false;
    }

    if (m_port.empty())
    {
        yCIError(XTR, id()) << "Invalid empty serial port";
        return false;
    }

    if (m_baudrate <= 0)
    {
        yCIError(XTR, id()) << "Invalid baudrate:" << m_baudrate;
        return false;
    }

    if (m_motorIds.size() != NUM_MOTORS)
    {
        yCIError(XTR, id()) << "Invalid number of motor IDs, expected" << NUM_MOTORS << "but got" << m_motorIds.size();
        return false;
    }

    if (std::unordered_set(m_motorIds.begin(), m_motorIds.end()).size() != m_motorIds.size())
    {
        yCIError(XTR, id()) << "Duplicate motor IDs found in the list";
        return false;
    }

    for (const auto & motorId : m_motorIds)
    {
        if (motorId <= 0)
        {
            yCIError(XTR, id()) << "Invalid motor ID in the list:" << motorId;
            return false;
        }
    }

    if (m_extraId < 0)
    {
        yCIError(XTR, id()) << "Invalid extra motor ID:" << m_extraId;
        return false;
    }
    else if (m_extraId != 0 && std::find(m_motorIds.begin(), m_motorIds.end(), m_extraId) != m_motorIds.end())
    {
        yCIError(XTR, id()) << "Extra motor ID" << m_extraId << "is already in the list of motor IDs";
        return false;
    }

    if (m_encoderPulses <= 0)
    {
        yCIError(XTR, id()) << "Invalid number of encoder pulses:" << m_encoderPulses;
        return false;
    }

    if (m_velocityRatio <= 0.0)
    {
        yCIError(XTR, id()) << "Invalid velocity ratio:" << m_velocityRatio;
        return false;
    }

    if (m_offsets.size() != NUM_MOTORS)
    {
        yCIError(XTR, id()) << "Invalid number of offsets, expected" << NUM_MOTORS << "but got" << m_offsets.size();
        return false;
    }

    if (m_period <= 0.0)
    {
        yCIError(XTR, id()) << "Invalid period:" << m_period;
        return false;
    }

    connector = std::make_unique<dynamixel::Connector>(m_port, m_baudrate);

    if (auto result = connector->broadcastPing(); result.isSuccess())
    {
        yCIInfo(XTR, id()) << "Broadcast ping successful, got IDs:" << result.value();
    }
    else
    {
        yCIError(XTR, id()) << "Broadcast ping failed:" << dynamixel::getErrorMessage(result.error());
        return false;
    }

    for (const auto & id : m_motorIds)
    {
        motors.push_back(connector->createMotor(id));
    }

    if (m_extraId != 0)
    {
        yCIInfo(XTR, id()) << "Using extra motor with ID" << m_extraId;
        motors.push_back(connector->createMotor(m_extraId)); // new size: NUM_MOTORS + 1

        if (m_extraReverse)
        {
            motors.back()->setDirection(dynamixel::Direction::REVERSE);
        }
    }

    for (const auto & motor : motors)
    {
        auto motorId = motor->getID();
        auto modelNumber = motor->getModelNumber();
        auto modelName = motor->getModelName();

        yCIInfo(XTR, id()) << "Got motor ID" << motorId << "with model number" << modelNumber << "and model name" << modelName;
    }

    lastJointPositions.resize(NUM_MOTORS);
    lastButtonStates.resize(NUM_BUTTONS);

    if (!yarp::os::PeriodicThread::setPeriod(m_period) || !yarp::os::PeriodicThread::start())
    {
        yCIError(XTR, id()) << "Failed to start periodic thread with period:" << m_period;
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
