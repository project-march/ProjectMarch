cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_robot_monitor)

find_package(catkin REQUIRED COMPONENTS
    diagnostic_aggregator
    diagnostic_updater
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    diagnostic_aggregator
    diagnostic_updater
    std_msgs
)

catkin_python_setup()

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
