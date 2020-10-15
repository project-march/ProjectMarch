cmake_minimum_required(VERSION 3.10.2)
project(march_simulation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/upload_joint_names scripts/to_world_transform
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch obstacles worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
