// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <cmath> // std::max

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

constexpr auto THROTTLE = 1.0; // [s]

// ------------------- IAxisInfo Related ------------------------------------

void XTrainer::run()
{
    std::lock_guard lock(mtx);

    // joints

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < NUM_MOTORS; i++)
    {
        executor->addCmd(motors[i]->stageGetPresentPosition());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCIErrorThrottle(XTR, id(), THROTTLE) << "Failed to get encoders:" << dynamixel::getErrorMessage(result.error());
    }
    else
    {
        for (auto i = 0; i < NUM_MOTORS; i++)
        {
            lastJointPositions[i] = result.value()[i].value() * 360.0 / m_encoderPulses - m_offsets[i];
        }

        // make the trigger range from exact 0.0
        lastJointPositions[NUM_AXES] = std::max(lastJointPositions[NUM_AXES], 0.0);
    }

    // buttons

    auto * portHandler = connector->getPortHandler();

    std::uint8_t txBuffer[] = {0xAA, 0x55, 0xAA};
    std::uint8_t rxBuffer[2];

    auto ret = portHandler->writePort(txBuffer, sizeof(txBuffer));

    if (ret <= 0)
    {
        yCIErrorThrottle(XTR, id(), THROTTLE) << "Failed to write to port for retrieving button state";
    }

    portHandler->setPacketTimeout(static_cast<std::uint16_t>(2)); // packet length

    while (true)
    {
        ret = portHandler->readPort(rxBuffer, sizeof(rxBuffer));

        if (ret >= 2 || portHandler->isPacketTimeout())
        {
            break;
        }
    }

    if (ret == 2 && (rxBuffer[0] | rxBuffer[1]) == 0xFF && (rxBuffer[0] & rxBuffer[1]) == 0x00)
    {
        lastButtonStates[0] = static_cast<double>((rxBuffer[0] >> 4) == 0x00 && (rxBuffer[1] >> 4) == 0x0F ? 1.0 : 0.0);
        lastButtonStates[1] = static_cast<double>((rxBuffer[0] & 0x0F) == 0x00 && (rxBuffer[1] & 0x0F) == 0x0F ? 1.0 : 0.0);
    }
    else
    {
        yCIWarningThrottle(XTR, id(), THROTTLE) << "Invalid button state received:" << rxBuffer[0] << rxBuffer[1] << ret;
    }
}

// -----------------------------------------------------------------------------
