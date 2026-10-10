// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IVelocityDirect Related --------------------------------
yarp::dev::ReturnValue XTrainerControlBoard::getNumberOfMotors(int * number)
{
    std::size_t axes;
    getAxes(axes);
    *number = static_cast<int>(axes);
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setRefDutyCycle(int m, double ref)
{
    CHECK_JOINT(m);

    if (auto result = motors[m]->setGoalPWM(ref); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set PWM duty cycle:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setRefDutyCycles(const double * refs)
{
    auto executor = connector->createGroupExecutor();

    for (int j = 0; j < motors.size(); j++)
    {
        executor->addCmd(motors[j]->stageSetGoalPWM(refs[j]));
    }

    if (auto result = executor->executeWrite(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to set PWM duty cycles:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefDutyCycle(int m, double * ref)
{
    CHECK_JOINT(m);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefDutyCycles(double * refs)
{
    bool ok = true;

    for (int j = 0; j < motors.size(); j++)
    {
        ok &= getRefDutyCycle(j, &refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getDutyCycle(int m, double * val)
{
    CHECK_JOINT(m);

    if (auto result = motors[m]->getPresentPWM(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get PWM duty cycle:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        *val = result.value();
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getDutyCycles(double * vals)
{
    auto executor = connector->createGroupExecutor();

    for (int j = 0; j < motors.size(); j++)
    {
        executor->addCmd(motors[j]->stageGetPresentPWM());
    }

    if (auto result = executor->executeRead(); !result.isSuccess())
    {
        yCError(XCB) << "Failed to get PWM duty cycles:" << dynamixel::getErrorMessage(result.error());
        return yarp::dev::ReturnValue_error_method_failed;
    }
    else
    {
        for (int j = 0; j < motors.size(); j++)
        {
            vals[j] = result.value()[j].value();
        }
    }

    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
