cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_gait_generator)

find_package(catkin REQUIRED COMPONENTS
    march_shared_classes
    march_shared_resources
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch resource test test/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/testing_gait_generator.test)
endif()
