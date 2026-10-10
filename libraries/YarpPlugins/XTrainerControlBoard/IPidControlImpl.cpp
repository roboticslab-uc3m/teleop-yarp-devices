// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPidControl Related --------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION,
        yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY
    };

    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    CHECK_JOINT(j);

    switch (pidtype)
    {
    case yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
    {
        if (auto result = motors[j]->setPositionPGain(pid.kp); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set position P gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setPositionIGain(pid.ki); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set position I gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setPositionDGain(pid.kd); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set position D gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        break;
    }
    case yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
    {
        if (auto result = motors[j]->setVelocityPGain(pid.kp); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set velocity P gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setVelocityIGain(pid.ki); !result.isSuccess())
        {
            yCError(XCB) << "Failed to set velocity I gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        break;
    }
    default:
        yCError(XCB) << "Invalid PID type";
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    bool ok = true;

    for (unsigned int i = 0; i < motors.size(); i++)
    {
        ok &= setPid(pidtype, i, pids[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidExtraInfo(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidExtraInfos(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------
