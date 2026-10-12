// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "XTrainerControlBoard.hpp"

// ------------------- IAxisInfo Related ------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getAxisName(int axis, std::string & name)
{
    CHECK_JOINT(axis);
    name = std::string("ID") + std::to_string(motors[axis]->getID());
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue XTrainerControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum & type)
{
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return yarp::dev::ReturnValue_ok;
}

// -----------------------------------------------------------------------------
