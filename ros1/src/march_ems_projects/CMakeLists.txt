cmake_minimum_required(VERSION 3.0.2)
project(march_ems_projects)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY sw_files
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
