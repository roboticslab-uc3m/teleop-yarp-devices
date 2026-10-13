// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPidControl Related --------------------------------

yarp::dev::ReturnValue XTrainer::getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum> &avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION,
        yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY};

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

bool XTrainer::setPidInternal(const yarp::dev::PidControlTypeEnum &pidtype, int j, const yarp::dev::Pid &pid)
{
    switch (pidtype)
    {
    case yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
    {
        if (auto result = motors[j]->setPositionPGain(pid.kp); !result.isSuccess())
        {
            yCIError(XTR, id()) << "Failed to set position P gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setPositionIGain(pid.ki); !result.isSuccess())
        {
            yCIError(XTR, id()) << "Failed to set position I gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setPositionDGain(pid.kd); !result.isSuccess())
        {
            yCIError(XTR, id()) << "Failed to set position D gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        break;
    }
    case yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
    {
        if (auto result = motors[j]->setVelocityPGain(pid.kp); !result.isSuccess())
        {
            yCIError(XTR, id()) << "Failed to set velocity P gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        if (auto result = motors[j]->setVelocityIGain(pid.ki); !result.isSuccess())
        {
            yCIError(XTR, id()) << "Failed to set velocity I gain:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }

        break;
    }
    default:
        yCIError(XTR, id()) << "Invalid PID type";
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

    return true;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPid(const yarp::dev::PidControlTypeEnum &pidtype, int j, const yarp::dev::Pid &pid)
{
    CHECK_JOINT(j);

    std::lock_guard lock(mtx);

    bool ok = setPidInternal(pidtype, j, pid);

    if (j == EXTRA_MOTOR_IDX && motors.size() > NUM_MOTORS)
    {
        ok &= setPidInternal(pidtype, NUM_MOTORS, pid);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPids(const yarp::dev::PidControlTypeEnum &pidtype, const yarp::dev::Pid *pids)
{
    bool ok = true;

    for (auto i = 0; i < NUM_AXES; i++)
    {
        ok &= setPid(pidtype, i, pids[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidReference(const yarp::dev::PidControlTypeEnum &pidtype, int j, double ref)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidReferences(const yarp::dev::PidControlTypeEnum &pidtype, const double *refs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidErrorLimit(const yarp::dev::PidControlTypeEnum &pidtype, int j, double limit)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidErrorLimits(const yarp::dev::PidControlTypeEnum &pidtype, const double *limits)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidError(const yarp::dev::PidControlTypeEnum &pidtype, int j, double *err)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidErrors(const yarp::dev::PidControlTypeEnum &pidtype, double *errs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidOutput(const yarp::dev::PidControlTypeEnum &pidtype, int j, double *out)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidOutputs(const yarp::dev::PidControlTypeEnum &pidtype, double *outs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPid(const yarp::dev::PidControlTypeEnum &pidtype, int j, yarp::dev::Pid *pid)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPids(const yarp::dev::PidControlTypeEnum &pidtype, yarp::dev::Pid *pids)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidOffset(const yarp::dev::PidControlTypeEnum &pidtype, int j, double &v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidFeedforward(const yarp::dev::PidControlTypeEnum &pidtype, int j, double &v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidExtraInfo(const yarp::dev::PidControlTypeEnum &pidtype, int j, yarp::dev::PidExtraInfo &info)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidExtraInfos(const yarp::dev::PidControlTypeEnum &pidtype, std::vector<yarp::dev::PidExtraInfo> &info)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidReference(const yarp::dev::PidControlTypeEnum &pidtype, int j, double *ref)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidReferences(const yarp::dev::PidControlTypeEnum &pidtype, double *refs)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidErrorLimit(const yarp::dev::PidControlTypeEnum &pidtype, int j, double *limit)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getPidErrorLimits(const yarp::dev::PidControlTypeEnum &pidtype, double *limits)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::resetPid(const yarp::dev::PidControlTypeEnum &pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::disablePid(const yarp::dev::PidControlTypeEnum &pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::enablePid(const yarp::dev::PidControlTypeEnum &pidtype, int j)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidOffset(const yarp::dev::PidControlTypeEnum &pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setPidFeedforward(const yarp::dev::PidControlTypeEnum &pidtype, int j, double v)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::isPidEnabled(const yarp::dev::PidControlTypeEnum &pidtype, int j, bool &enabled)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------
