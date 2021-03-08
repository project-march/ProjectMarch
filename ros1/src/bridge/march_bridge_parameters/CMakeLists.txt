cmake_minimum_required(VERSION 3.0.2)
project(march_bridge_parameters)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
