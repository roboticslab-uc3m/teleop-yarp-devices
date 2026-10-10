// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

// ------------------- IControlMode Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);
    avail = {};
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
{
    CHECK_JOINT(j);
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
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
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
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
