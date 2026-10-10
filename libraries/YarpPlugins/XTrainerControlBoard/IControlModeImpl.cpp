// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

namespace
{
    yarp::dev::ControlModeEnum dynamixelToYarpControlMode(dynamixel::OperatingMode mode)
    {
        switch (mode)
        {
        case dynamixel::OperatingMode::POSITION:
            return yarp::dev::ControlModeEnum::VOCAB_CM_POSITION;
        case dynamixel::OperatingMode::VELOCITY:
            return yarp::dev::ControlModeEnum::VOCAB_CM_VELOCITY;
        case dynamixel::OperatingMode::PWM:
            return yarp::dev::ControlModeEnum::VOCAB_CM_PWM;
        case dynamixel::OperatingMode::CURRENT:
            return yarp::dev::ControlModeEnum::VOCAB_CM_CURRENT;
        default:
            return yarp::dev::ControlModeEnum::VOCAB_CM_UNKNOWN;
        }
    }

    dynamixel::OperatingMode yarpToDynamixelControlMode(yarp::dev::SelectableControlModeEnum mode)
    {
        switch (mode)
        {
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION:
            return dynamixel::OperatingMode::POSITION;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY:
            return dynamixel::OperatingMode::VELOCITY;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_PWM:
            return dynamixel::OperatingMode::PWM;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_CURRENT:
            return dynamixel::OperatingMode::CURRENT;
        default:
            return dynamixel::OperatingMode::POSITION; // default fallback
        }
    }
}

// ------------------- IControlMode Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_PWM,
    };

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
{
    CHECK_JOINT(j);

    auto result = motors[j]->getOperatingMode();

    if (!result.isSuccess())
    {
        yCError(XCB) << "Failed to get control mode:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    mode = dynamixelToYarpControlMode(result.value());
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
{
    bool ok = true;

    modes.resize(m_axes);

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= getControlMode(i, modes[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
{
    bool ok = true;

    modes.resize(joints.size());

    for (size_t i = 0; i < joints.size(); i++)
    {
        ok &= getControlMode(joints[i], modes[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
{
    CHECK_JOINT(j);

    auto dMode = yarpToDynamixelControlMode(mode);
    auto result = motors[j]->setOperatingMode(dMode);

    if (!result.isSuccess())
    {
        yCError(XCB) << "Failed to set control mode:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    bool ok = true;

    for (int j = 0; j < joints.size(); j++)
    {
        ok &= setControlMode(joints[j], modes[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
