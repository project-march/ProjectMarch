cmake_minimum_required(VERSION 3.0.2)
project(march_parameter_server)

find_package(catkin REQUIRED COMPONENTS
    march_shared_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/launch/test_parameter_server.test)
endif()