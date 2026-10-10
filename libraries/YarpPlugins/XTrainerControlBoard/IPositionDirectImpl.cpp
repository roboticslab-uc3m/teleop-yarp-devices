// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

// ------------------- IPositionDirect Related --------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPosition(int j, double ref)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(const double * refs)
{
    bool ok = true;

    for (int j = 0; j < m_axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
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

    for (int j = 0; j < m_axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
}

// -----------------------------------------------------------------------------
