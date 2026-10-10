if(NOT dynamixel_sdk_FOUND AND (NOT DEFINED ENABLE_dynamixel_sdk OR ENABLE_dynamixel_sdk))
    message(WARNING "dynamixel_sdk package not found, disabling XTrainerControlBoard")
endif()

yarp_prepare_plugin(XTrainerControlBoard
                    CATEGORY device
                    TYPE XTrainerControlBoard
                    INCLUDE XTrainerControlBoard.hpp
                    DEFAULT ON
                    DEPENDS "dynamixel_sdk_FOUND;YARP_VERSION VERSION_GREATER_EQUAL 4.0"
                    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp
                    GENERATE_PARSER)

if(NOT SKIP_XTrainerControlBoard)

    yarp_add_plugin(XTrainerControlBoard)

    target_sources(XTrainerControlBoard PRIVATE XTrainerControlBoard.hpp
                                                DeviceDriverImpl.cpp
                                                IControlLimitsImpl.cpp
                                                IControlModeImpl.cpp
                                                IEncodersImpl.cpp
                                                IPositionDirectImpl.cpp
                                                XTrainerControlBoard_ParamsParser.h
                                                XTrainerControlBoard_ParamsParser.cpp
                                                LogComponent.hpp
                                                LogComponent.cpp)

    target_link_libraries(XTrainerControlBoard YARP::YARP_os
                                               YARP::YARP_dev
                                               dynamixel_sdk::dynamixel_sdk)

    yarp_install(TARGETS XTrainerControlBoard
                 LIBRARY DESTINATION ${TELEOP-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${TELEOP-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${TELEOP-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_XTrainerControlBoard OFF CACHE BOOL "Enable/disable XTrainerControlBoard device" FORCE)

endif()
