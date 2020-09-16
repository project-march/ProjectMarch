cmake_minimum_required(VERSION 3.0.2)
project(march_gait_files)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY minimal airgait-v test_versions-v training-v
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
