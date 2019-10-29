cmake_minimum_required(VERSION 2.8.3)
project(march_gait_files)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY airgait test_versions training airgait-v test_versions-v training-v
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
