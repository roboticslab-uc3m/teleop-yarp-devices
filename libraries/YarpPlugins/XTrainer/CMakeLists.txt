if(NOT dynamixel_sdk_FOUND AND (NOT DEFINED ENABLE_dynamixel_sdk OR ENABLE_dynamixel_sdk))
    message(WARNING "dynamixel_sdk package not found, disabling XTrainer")
endif()

yarp_prepare_plugin(XTrainer
                    CATEGORY device
                    TYPE XTrainer
                    INCLUDE XTrainer.hpp
                    DEFAULT ON
                    DEPENDS "dynamixel_sdk_FOUND;YARP_VERSION VERSION_GREATER_EQUAL 4.0"
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp
                    GENERATE_PARSER)

if(NOT SKIP_XTrainer)

    yarp_add_plugin(XTrainer)

    target_sources(XTrainer PRIVATE XTrainer.hpp
                                    DeviceDriverImpl.cpp
                                    IAxisInfoImpl.cpp
                                    IControlLimitsImpl.cpp
                                    IControlModeImpl.cpp
                                    IEncodersImpl.cpp
                                    IPositionDirectImpl.cpp
                                    IPidControlImpl.cpp
                                    IPWMControlImpl.cpp
                                    IVelocityDirectImpl.cpp
                                    IJoypadControllerImpl.cpp
                                    XTrainer_ParamsParser.h
                                    XTrainer_ParamsParser.cpp
                                    LogComponent.hpp
                                    LogComponent.cpp)

    target_link_libraries(XTrainer YARP::YARP_os
                                   YARP::YARP_dev
                                   dynamixel_sdk::dynamixel_sdk)

    yarp_install(TARGETS XTrainer
                 LIBRARY DESTINATION ${TELEOP-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${TELEOP-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${TELEOP-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_XTrainer OFF CACHE BOOL "Enable/disable XTrainer device" FORCE)

endif()
