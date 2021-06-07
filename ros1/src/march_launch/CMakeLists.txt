cmake_minimum_required(VERSION 3.0.2)
project(march_launch)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/upload_controller_values.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
