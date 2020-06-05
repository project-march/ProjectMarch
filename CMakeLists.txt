cmake_minimum_required(VERSION 2.8.3)
project(march_tutorials)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY doc/useful_tools/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
