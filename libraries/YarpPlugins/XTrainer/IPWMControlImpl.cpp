// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IVelocityDirect Related --------------------------------
yarp::dev::ReturnValue XTrainer::getNumberOfMotors(int * number)
{
    *number = NUM_AXES;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setRefDutyCycle(int m, double ref)
{
    CHECK_JOINT(m);

    std::lock_guard lock(mtx);

    if (m == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
    {
        auto executor = connector->createGroupExecutor();
        executor->addCmd(motors[m]->stageSetGoalPWM(ref));
        executor->addCmd(motors[NUM_AXES]->stageSetGoalPWM(ref));

        if (auto result = executor->executeWrite(); !result.isSuccess())
        {
            yCError(XTR) << "Failed to set PWM duty cycle for dual-motor axis:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }
    else
    {
        if (auto result = motors[m]->setGoalPWM(ref); !result.isSuccess())
        {
            yCError(XTR) << "Failed to set PWM duty cycle:" << dynamixel::getErrorMessage(result.error());
            return yarp::dev::ReturnValue_error_method_failed;
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::setRefDutyCycles(const double * refs)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto j = 0; j < NUM_AXES; j++)
    {
        executor->addCmd(motors[j]->stageSetGoalPWM(refs[j]));

        if (j == EXTRA_MOTOR_IDX && motors.size() > NUM_AXES)
        {
            executor->addCmd(motors[NUM_AXES]->stageSetGoalPWM(refs[j]));
        }
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XTR) << "Failed to set PWM duty cycles:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getRefDutyCycle(int m, double * ref)
{
    CHECK_JOINT(m);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getRefDutyCycles(double * refs)
{
    bool ok = true;

    for (auto j = 0; j < NUM_AXES; j++)
    {
        ok &= getRefDutyCycle(j, &refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getDutyCycle(int m, double * val)
{
    CHECK_JOINT(m);

    std::lock_guard lock(mtx);

    if (auto result = motors[m]->getPresentPWM(); !result.isSuccess())
    {
        yCError(XTR) << "Failed to get PWM duty cycle:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *val = result.value();
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getDutyCycles(double * vals)
{
    std::lock_guard lock(mtx);

    auto executor = connector->createGroupExecutor();

    for (auto j = 0; j < NUM_AXES; j++)
    {
        executor->addCmd(motors[j]->stageGetPresentPWM());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCError(XTR) << "Failed to get PWM duty cycles:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        for (auto j = 0; j < NUM_AXES; j++)
        {
            vals[j] = result.value()[j].value();
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
