// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __XTRAINER_CONTROL_BOARD_HPP__
#define __XTRAINER_CONTROL_BOARD_HPP__

#include <memory>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IVelocityDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <dynamixel_easy_sdk/dynamixel_easy_sdk.hpp>

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
                             public yarp::dev::IAxisInfo,
                             public yarp::dev::IControlLimits,
                             public yarp::dev::IControlMode,
                             public yarp::dev::IEncodersTimed,
                             public yarp::dev::IPidControl,
                             public yarp::dev::IPositionDirect,
                             public yarp::dev::IPWMControl,
                             public yarp::dev::IVelocityDirect,
                             public XTrainerControlBoard_ParamsParser
{
public:
    // --------- IAxisInfo Declarations. Implementation in IAxisInfoImpl.cpp ---------
    yarp::dev::ReturnValue getAxisName(int axis, std::string & name) override;
    yarp::dev::ReturnValue getJointType(int axis, yarp::dev::JointTypeEnum & type) override;

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

    // ---------- IEncodersTimed Declarations. Implementation in IEncoderImpl.cpp ----------
    yarp::dev::ReturnValue getAxes(std::size_t & axes) override;
    yarp::dev::ReturnValue resetEncoder(int j) override;
    yarp::dev::ReturnValue resetEncoders() override;
    yarp::dev::ReturnValue setEncoder(int j, double val) override;
    yarp::dev::ReturnValue setEncoders(const double * vals) override;
    yarp::dev::ReturnValue getEncoder(int j, double * v) override;
    yarp::dev::ReturnValue getEncoders(double * encs) override;
    yarp::dev::ReturnValue getEncoderTimed(int j, double * encs, double * time) override;
    yarp::dev::ReturnValue getEncodersTimed(double * encs, double * time) override;
    yarp::dev::ReturnValue getEncoderSpeed(int j, double * sp) override;
    yarp::dev::ReturnValue getEncoderSpeeds(double * spds) override;
    yarp::dev::ReturnValue getEncoderAcceleration(int j, double * spds) override;
    yarp::dev::ReturnValue getEncoderAccelerations(double * accs) override;

    // ------- IPidControl declarations. Implementation in IPidControlImpl.cpp -------
    yarp::dev::ReturnValue getAvailablePids(int j, std::vector<yarp::dev::PidControlTypeEnum> & avail) override;
    yarp::dev::ReturnValue setPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid) override;
    yarp::dev::ReturnValue setPids(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids) override;
    yarp::dev::ReturnValue setPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref) override;
    yarp::dev::ReturnValue setPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs) override;
    yarp::dev::ReturnValue setPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit) override;
    yarp::dev::ReturnValue setPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits) override;
    yarp::dev::ReturnValue getPidError(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err) override;
    yarp::dev::ReturnValue getPidErrors(const yarp::dev::PidControlTypeEnum & pidtype, double * errs) override;
    yarp::dev::ReturnValue getPidOutput(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out) override;
    yarp::dev::ReturnValue getPidOutputs(const yarp::dev::PidControlTypeEnum & pidtype, double * outs) override;
    yarp::dev::ReturnValue getPid(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid) override;
    yarp::dev::ReturnValue getPids(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids) override;
    yarp::dev::ReturnValue getPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    yarp::dev::ReturnValue getPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v) override;
    yarp::dev::ReturnValue getPidExtraInfo(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info) override;
    yarp::dev::ReturnValue getPidExtraInfos(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info) override;
    yarp::dev::ReturnValue getPidReference(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref) override;
    yarp::dev::ReturnValue getPidReferences(const yarp::dev::PidControlTypeEnum & pidtype, double * refs) override;
    yarp::dev::ReturnValue getPidErrorLimit(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit) override;
    yarp::dev::ReturnValue getPidErrorLimits(const yarp::dev::PidControlTypeEnum & pidtype, double * limits) override;
    yarp::dev::ReturnValue resetPid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    yarp::dev::ReturnValue disablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    yarp::dev::ReturnValue enablePid(const yarp::dev::PidControlTypeEnum & pidtype, int j) override;
    yarp::dev::ReturnValue setPidOffset(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    yarp::dev::ReturnValue setPidFeedforward(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v) override;
    yarp::dev::ReturnValue isPidEnabled(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled) override;

    // ------- IPositionDirect declarations. Implementation in IPositionDirectImpl.cpp -------
    yarp::dev::ReturnValue setPosition(int j, double ref) override;
    yarp::dev::ReturnValue setPositions(int n_joint, const int * joints, const double * refs) override;
    yarp::dev::ReturnValue setPositions(const double * refs) override;
    yarp::dev::ReturnValue getRefPosition(int joint, double * ref) override;
    yarp::dev::ReturnValue getRefPositions(double * refs) override;
    yarp::dev::ReturnValue getRefPositions(int n_joint, const int * joints, double * refs) override;

    // ------- IPWMControl declarations. Implementation in IPWMControlImpl.cpp -------
    yarp::dev::ReturnValue getNumberOfMotors(int * number) override;
    yarp::dev::ReturnValue setRefDutyCycle(int m, double ref) override;
    yarp::dev::ReturnValue setRefDutyCycles(const double * refs) override;
    yarp::dev::ReturnValue getRefDutyCycle(int m, double * ref) override;
    yarp::dev::ReturnValue getRefDutyCycles(double * refs) override;
    yarp::dev::ReturnValue getDutyCycle(int m, double * val) override;
    yarp::dev::ReturnValue getDutyCycles(double * vals) override;

    // ------- IVelocityDirect declarations. Implementation in IVelocityDirectImpl.cpp -------
    yarp::dev::ReturnValue setRefVelocity(int jnt, double vel) override;
    yarp::dev::ReturnValue setRefVelocity(const std::vector<double> & vels) override;
    yarp::dev::ReturnValue setRefVelocity(const std::vector<int> & jnts, const std::vector<double> & vels) override;
    yarp::dev::ReturnValue getRefVelocity(const int jnt, double & vel) override;
    yarp::dev::ReturnValue getRefVelocity(std::vector<double> & vels) override;
    yarp::dev::ReturnValue getRefVelocity(const std::vector<int> & jnts, std::vector<double> & vels) override;

    // -------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp --------
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

private:
    std::unique_ptr<dynamixel::Connector> connector {nullptr};
    std::vector<std::unique_ptr<dynamixel::Motor>> motors;
};

#endif // __XTRAINER_CONTROL_BOARD_HPP__
