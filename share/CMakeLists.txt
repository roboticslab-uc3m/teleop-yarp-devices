if(ENABLE_SpaceNavigator)
    yarp_install(FILES applications/spaceNavigator.xml
                 DESTINATION ${TELEOP-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
endif()

if(ENABLE_XTrainerControlBoard)
    yarp_install(DIRECTORY contexts/kinematics
                           contexts/xtrainer
                 DESTINATION ${TELEOP-YARP-DEVICES_CONTEXTS_INSTALL_DIR})
endif()

yarp_install(FILES applications/ymanager.ini
             DESTINATION ${TELEOP-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
