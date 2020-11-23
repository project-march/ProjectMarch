cmake_minimum_required(VERSION 3.0.2)
project(march_gait_files)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY airgait-v minimal simulation_only-vi test_versions-vi training-v
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
