// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <numeric> // std::iota

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPositionDirect Related --------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPosition(int j, double ref)
{
    CHECK_JOINT(j);

    if (j == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
    {
        std::vector indices = {j};
        std::vector values = {ref};
        return setPositions(indices.size(), indices.data(), values.data());
    }

    std::lock_guard lock(mtx);

    if (auto result = motors[j]->setGoalPosition(ref * m_encoderPulses / 360.0); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set position:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(const double * refs)
{
    std::vector<int> indices(NUM_AXES);
    std::iota(indices.begin(), indices.end(), 0);
    return setPositions(NUM_AXES, indices.data(), refs);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto i = 0; i < n_joint; i++)
    {
        executor->addCmd(motors[joints[i]]->stageSetGoalPosition(refs[i] * m_encoderPulses / 360.0));

        if (joints[i] == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
        {
            executor->addCmd(motors[NUM_AXES]->stageSetGoalPosition((refs[i] + m_extraOffset) * m_encoderPulses / 360.0));
        }
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set positions:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPosition(int joint, double * ref)
{
    CHECK_JOINT(joint);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPositions(double * refs)
{
    bool ok = true;

    for (auto j = 0; j < NUM_AXES; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    bool ok = true;

    for (auto i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
