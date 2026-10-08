# FIXME: workaround for YARP 3.12+.
option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Allow YARP to (re)build device param parsers" ON)

# YARP devices.
# deprecated, see https://github.com/roboticslab-uc3m/yarp-devices/issues/265
# add_subdirectory(LeapMotionSensor)
add_subdirectory(SpaceNavigator)
add_subdirectory(Wiimote)
