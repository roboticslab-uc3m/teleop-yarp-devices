// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <cmath> // std::max

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IAxisInfo Related ------------------------------------

void XTrainer::run()
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < NUM_MOTORS; i++)
    {
        executor->addCmd(motors[i]->stageGetPresentPosition());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCError(XTR) << "Failed to get encoders:" << dynamixel::getErrorMessage(result.error());
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
}

// -----------------------------------------------------------------------------
