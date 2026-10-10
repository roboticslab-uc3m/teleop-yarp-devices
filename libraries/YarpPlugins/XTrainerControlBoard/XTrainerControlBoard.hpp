// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __XTRAINER_CONTROL_BOARD_HPP__
#define __XTRAINER_CONTROL_BOARD_HPP__

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "XTrainerControlBoard_ParamsParser.h"

#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= m_axes) return yarp::dev::ReturnValue_error_input_out_of_bounds; } while (0)

/**
 * @ingroup YarpPlugins
 * @defgroup XTrainerControlBoard
 * @brief Contains XTrainerControlBoard.
 */

/**
 * @ingroup XTrainerControlBoard
 * @brief Implements several motor interfaces.
 */
class XTrainerControlBoard : public yarp::dev::DeviceDriver,
                             public yarp::dev::IControlLimits,
                             public yarp::dev::IControlMode,
                             public yarp::dev::IEncodersTimed,
                             public yarp::dev::IPositionDirect,
                             public XTrainerControlBoard_ParamsParser
{
public:
    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------
    yarp::dev::ReturnValue setPosition(int j, double ref) override;
    yarp::dev::ReturnValue setPositions(int n_joint, const int * joints, const double * refs) override;
    yarp::dev::ReturnValue setPositions(const double * refs) override;
    yarp::dev::ReturnValue getRefPosition(int joint, double * ref) override;
    yarp::dev::ReturnValue getRefPositions(double * refs) override;
    yarp::dev::ReturnValue getRefPositions(int n_joint, const int * joints, double * refs) override;

    // ---------- IEncodersTimed Declarations. Implementation in IEncoderImpl.cpp ----------
    yarp::dev::ReturnValue getAxes(std::size_t & axes) override;
    yarp::dev::ReturnValue resetEncoder(int j) override;
    yarp::dev::ReturnValue resetEncoders() override;
    yarp::dev::ReturnValue setEncoder(int j, double val) override;
    yarp::dev::ReturnValue setEncoders(const double * vals) override;
    yarp::dev::ReturnValue getEncoder(int j, double * v) override;
    yarp::dev::ReturnValue getEncoders(double * encs) override;
    yarp::dev::ReturnValue getEncodersTimed(double * encs, double * time) override;
    yarp::dev::ReturnValue getEncoderTimed(int j, double * encs, double * time) override;
    yarp::dev::ReturnValue getEncoderSpeed(int j, double * sp) override;
    yarp::dev::ReturnValue getEncoderSpeeds(double * spds) override;
    yarp::dev::ReturnValue getEncoderAcceleration(int j, double * spds) override;
    yarp::dev::ReturnValue getEncoderAccelerations(double * accs) override;

    // --------- IControlLimits Declarations. Implementation in IControlLimitsImpl.cpp ---------
    yarp::dev::ReturnValue setPosLimits(int axis, double min, double max) override;
    yarp::dev::ReturnValue getPosLimits(int axis, double * min, double * max) override;
    yarp::dev::ReturnValue setVelLimits(int axis, double min, double max) override;
    yarp::dev::ReturnValue getVelLimits(int axis, double * min, double * max) override;

    // --------- IControlMode Declarations. Implementation in IControlModeImpl.cpp ---------
    yarp::dev::ReturnValue getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail) override;
    yarp::dev::ReturnValue getControlMode(int j, yarp::dev::ControlModeEnum & mode) override;
    yarp::dev::ReturnValue getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlMode(int j, yarp::dev::SelectableControlModeEnum mode) override;
    yarp::dev::ReturnValue setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;
    yarp::dev::ReturnValue setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
};

#endif // __XTRAINER_CONTROL_BOARD_HPP__
