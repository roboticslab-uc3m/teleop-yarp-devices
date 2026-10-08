if(ENABLE_SpaceNavigator)
    yarp_install(FILES applications/spaceNavigator.xml
                 DESTINATION ${TELEOP-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
endif()

yarp_install(FILES applications/ymanager.ini
             DESTINATION ${TELEOP-YARP-DEVICES_APPLICATIONS_INSTALL_DIR})
