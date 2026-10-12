// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

// ------------------- IJoypadController Related ------------------------------------

yarp::dev::ReturnValue XTrainer::getAxisCount(std::size_t & axis_count)
{
    axis_count = 6;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getButtonCount(std::size_t & button_count)
{
    button_count = 0;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getTrackballCount(std::size_t & trackball_count)
{
    trackball_count = 0;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getHatCount(std::size_t & hat_count)
{
    hat_count = 0;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getTouchSurfaceCount(std::size_t & touch_count)
{
    touch_count = 0;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getStickCount(std::size_t & stick_count)
{
    stick_count = 0;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getButton(std::size_t button_id, double & value)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getTrackball(std::size_t trackball_id, yarp::dev::TrackballData & value)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getHat(std::size_t hat_id, unsigned char & value)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getAxis(std::size_t axis_id, double & value)
{
    CHECK_JOINT(axis_id);
    std::lock_guard lock(mtx);
    value = lastJointPositions[axis_id];
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getAllAxes(std::vector<double> & values)
{
    std::lock_guard lock(mtx);
    values = lastJointPositions;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getStick(std::size_t stick_id, yarp::dev::StickData & value, JoypadCtrl_coordinateMode coordinate_mode)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getTouch(std::size_t touch_id, std::vector<yarp::dev::TouchData> & value)
{
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------
