// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <algorithm> // std::find
#include <numeric> // std::iota

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab32.h>

#include "LogComponent.hpp"

namespace
{
    const std::vector<yarp::dev::SelectableControlModeEnum> availableControlModes = {
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_PWM,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE,
    };

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
            return dynamixel::OperatingMode::EXTENDED_POSITION; // tricky, but eases if-branching in setControlMode()
        }
    }
}

// ------------------- IControlMode Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);
    avail = availableControlModes;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
{
    CHECK_JOINT(j);

    if (motors[j]->getTorqueStatus() == 1)
    {
        auto modeStatus = motors[j]->getOperatingModeStatus();
        mode = dynamixelToYarpControlMode(modeStatus);
    }
    else
    {
        mode = yarp::dev::ControlModeEnum::VOCAB_CM_IDLE;
    }

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

    if (std::find(availableControlModes.begin(), availableControlModes.end(), mode) == availableControlModes.end())
    {
        yCError(XCB) << "Control mode not available:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

    const auto dxlMode = yarpToDynamixelControlMode(mode);
    const bool dxlModeAlreadySet = dxlMode == motors[j]->getOperatingModeStatus();

    if (mode == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE && motors[j]->getTorqueStatus() == 0 ||
        dxlModeAlreadySet && motors[j]->getTorqueStatus() == 1)
    {
        return yarp::dev::ReturnValue_ok;
    }

    if (motors[j]->getTorqueStatus() == 1 && !dxlModeAlreadySet)
    {
        if (auto result = motors[j]->disableTorque(); !result.isSuccess())
        {
            yCError(XCB) << "Failed to disable torque:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    if (mode != yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE && !dxlModeAlreadySet)
    {
        if (auto result = motors[j]->setOperatingMode(dxlMode); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set control mode:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->enableTorque(); !result.isSuccess())
        {
            yCError(XCB) << "Failed to enable torque:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    std::vector<int> joints(motors.size());
    std::iota(joints.begin(), joints.end(), 0);
    return setControlModes(joints, modes);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    for (const auto & mode : modes)
    {
        if (std::find(availableControlModes.begin(), availableControlModes.end(), mode) == availableControlModes.end())
        {
            yCError(XCB) << "Control mode not available:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));
            return yarp::dev::ReturnValue_error_input_out_of_bounds;
        }
    }

    std::vector<dynamixel::OperatingMode> dxlModes;

    for (const auto & mode : modes)
    {
        dxlModes.push_back(yarpToDynamixelControlMode(mode));
    }

    auto executorOff = connector->createGroupExecutor();

    for (auto j = 0; j < joints.size(); j++)
    {
        const bool idleModeRequested = modes[j] == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE;
        const bool dxlModeAlreadySet = dxlModes[j] == motors[joints[j]]->getOperatingModeStatus();

        if (motors[joints[j]]->getTorqueStatus() == 1 && (idleModeRequested || !dxlModeAlreadySet))
        {
            executorOff->addCmd(motors[joints[j]]->stageDisableTorque());
        }
    }

    if (!executorOff->getStagedWriteCommands().empty())
    {
        if (auto result = executorOff->executeWrite(); !result.isSuccess())
        {
            yCError(XCB) << "Failed to disable torque (staged):" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    if (std::all_of(modes.begin(), modes.end(), [](auto mode) { return mode == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE; }))
    {
        return yarp::dev::ReturnValue_ok;
    }

    for (auto j = 0; j < joints.size(); j++)
    {
        if (dxlModes[j] != motors[joints[j]]->getOperatingModeStatus() && modes[j] != yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE)
        {
            if (auto result = motors[joints[j]]->setOperatingMode(dxlModes[j]); !result.isSuccess())
            {
                yCError(XCB) << "Failed to set control mode:" << dynamixel::getErrorMessage(result.error());
                return yarp::dev::ReturnValue_error_method_failed;
            }
        }
    }

    auto executorOn = connector->createGroupExecutor();

    for (auto j = 0; j < joints.size(); j++)
    {
        if (motors[joints[j]]->getTorqueStatus() == 0 && modes[j] != yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE)
        {
            executorOn->addCmd(motors[joints[j]]->stageEnableTorque());
        }
    }

    if (!executorOn->getStagedWriteCommands().empty())
    {
        if (auto result = executorOn->executeWrite(); !result.isSuccess())
        {
            yCError(XCB) << "Failed to enable torque (staged):" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
