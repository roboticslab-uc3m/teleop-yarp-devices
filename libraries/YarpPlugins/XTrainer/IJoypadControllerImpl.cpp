// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainer.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IJoypadController Related ------------------------------------

yarp::dev::ReturnValue XTrainer::getAxisCount(std::size_t & axis_count)
{
    axis_count = NUM_MOTORS;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainer::getButtonCount(std::size_t & button_count)
{
    button_count = NUM_BUTTONS;
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
    if (button_id < 0 || button_id >= NUM_BUTTONS)
    {
        yCIError(XTR, id()) << "Button ID out of bounds:" << button_id;
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

    std::lock_guard lock(mtx);
    value = lastButtonStates[button_id];
    return yarp::dev::ReturnValue_ok;
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
    if (axis_id < 0 || axis_id >= NUM_MOTORS)
    {
        yCIError(XTR, id()) << "Axis ID out of bounds:" << axis_id;
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
    }

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
