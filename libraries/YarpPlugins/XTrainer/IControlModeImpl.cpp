// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

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
            return yarp::dev::ControlModeEnum::VOCAB_CM_POSITION_DIRECT;
        case dynamixel::OperatingMode::VELOCITY:
            return yarp::dev::ControlModeEnum::VOCAB_CM_VELOCITY_DIRECT;
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
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT:
            return dynamixel::OperatingMode::POSITION;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY_DIRECT:
            return dynamixel::OperatingMode::VELOCITY;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_PWM:
            return dynamixel::OperatingMode::PWM;
        case yarp::dev::SelectableControlModeEnum::VOCAB_CM_CURRENT:
            return dynamixel::OperatingMode::CURRENT;
        default:
            // there is no UNKNOWN mode on the dxl side
            return dynamixel::OperatingMode::CURRENT_BASED_POSITION;
        }
    }
}

// ------------------- IControlMode Related ------------------------------------

yarp::dev::ReturnValue XTrainer::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);
    avail = availableControlModes;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
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

yarp::dev::ReturnValue XTrainer::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
{
    bool ok = true;

    modes.resize(NUM_AXES);

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= getControlMode(i, modes[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
{
    bool ok = true;

    modes.resize(joints.size());

    for (auto i = 0; i < joints.size(); i++)
    {
        ok &= getControlMode(joints[i], modes[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
{
    CHECK_JOINT(j);

    if (j == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
    {
        std::vector indices = {j};
        std::vector modes = {mode};
        return setControlModes(indices, modes);
    }

    if (std::find(availableControlModes.begin(), availableControlModes.end(), mode) == availableControlModes.end())
    {
        yCError(XTR) << "Control mode not available:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

    yCDebug(XTR) << "Setting control mode of joint" << j << "to" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));

    std::lock_guard lock(mtx);

    const auto dxlMode = yarpToDynamixelControlMode(mode);
    const bool idleModeRequested = mode == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE;
    const bool dxlModeAlreadySet = dxlMode == motors[j]->getOperatingModeStatus();

    if (idleModeRequested && motors[j]->getTorqueStatus() == 0 || dxlModeAlreadySet && motors[j]->getTorqueStatus() == 1)
    {
        return yarp::dev::ReturnValue_ok;
    }

    if (motors[j]->getTorqueStatus() == 1 && (idleModeRequested || !dxlModeAlreadySet))
    {
        if (auto result = motors[j]->disableTorque(); !result.isSuccess())
        {
            yCError(XTR) << "Failed to disable torque:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    if (!idleModeRequested && !dxlModeAlreadySet)
    {
        if (auto result = motors[j]->setOperatingMode(dxlMode); !result.isSuccess())
        {
            yCError(XTR) << "Failed to set control mode:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        yCInfo(XTR) << "Control mode of joint" << j << "set to" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));
    }

    if (motors[j]->getTorqueStatus() == 0 && !idleModeRequested)
    {
        if (auto result = motors[j]->enableTorque(); !result.isSuccess())
        {
            yCError(XTR) << "Failed to enable torque:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    std::vector<int> joints(NUM_AXES);
    std::iota(joints.begin(), joints.end(), 0);
    return setControlModes(joints, modes);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
{
    for (const auto & mode : modes)
    {
        if (std::find(availableControlModes.begin(), availableControlModes.end(), mode) == availableControlModes.end())
        {
            yCError(XTR) << "Control mode not available:" << yarp::os::Vocab32::decode(static_cast<yarp::conf::vocab32_t>(mode));
            return yarp::dev::ReturnValue_error_input_out_of_bounds;
        }
    }

    std::vector<dynamixel::OperatingMode> dxlModes;

    for (const auto & mode : modes)
    {
        dxlModes.push_back(yarpToDynamixelControlMode(mode));
    }

    std::lock_guard lock(mtx);

    auto executorOff = connector->createGroupExecutor();

    for (auto j = 0; j < joints.size(); j++)
    {
        const bool idleModeRequested = modes[j] == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE;
        const bool dxlModeAlreadySet = dxlModes[j] == motors[joints[j]]->getOperatingModeStatus();

        if (motors[joints[j]]->getTorqueStatus() == 1 && (idleModeRequested || !dxlModeAlreadySet))
        {
            executorOff->addCmd(motors[joints[j]]->stageDisableTorque());

            if (joints[j] == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
            {
                executorOff->addCmd(motors[NUM_AXES]->stageDisableTorque());
            }
        }
    }

    if (!executorOff->getStagedWriteCommands().empty())
    {
        if (auto result = executorOff->executeWrite(); !result.isSuccess())
        {
            yCError(XTR) << "Failed to disable torque (staged):" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    if (std::all_of(modes.begin(), modes.end(), [](auto mode) { return mode == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE; }))
    {
        auto decoded = static_cast<yarp::conf::vocab32_t>(yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE);
        yCInfo(XTR) << "Control mode of all selected joints set to" << yarp::os::Vocab32::decode(decoded);
        return yarp::dev::ReturnValue_ok;
    }

    for (auto j = 0; j < joints.size(); j++)
    {
        const bool idleModeRequested = modes[j] == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE;
        const bool dxlModeAlreadySet = dxlModes[j] == motors[joints[j]]->getOperatingModeStatus();

        if (!idleModeRequested && !dxlModeAlreadySet)
        {
            if (auto result = motors[joints[j]]->setOperatingMode(dxlModes[j]); !result.isSuccess())
            {
                yCError(XTR) << "Failed to set control mode:" << dynamixel::getErrorMessage(result.error());
                return yarp::dev::ReturnValue_error_method_failed;
            }

            if (joints[j] == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
            {
                if (auto result = motors[NUM_AXES]->setOperatingMode(dxlModes[j]); !result.isSuccess())
                {
                    yCError(XTR) << "Failed to set control mode for extra motor:" << dynamixel::getErrorMessage(result.error());
                    return yarp::dev::ReturnValue_error_method_failed;
                }
            }
        }
    }

    auto executorOn = connector->createGroupExecutor();

    for (auto j = 0; j < joints.size(); j++)
    {
        const bool idleModeRequested = modes[j] == yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE;

        if (motors[joints[j]]->getTorqueStatus() == 0 && !idleModeRequested)
        {
            executorOn->addCmd(motors[joints[j]]->stageEnableTorque());

            if (joints[j] == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
            {
                executorOn->addCmd(motors[NUM_AXES]->stageEnableTorque());
            }
        }
    }

    if (!executorOn->getStagedWriteCommands().empty())
    {
        if (auto result = executorOn->executeWrite(); !result.isSuccess())
        {
            yCError(XTR) << "Failed to enable torque (staged):" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
