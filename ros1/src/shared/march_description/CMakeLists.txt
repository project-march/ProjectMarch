cmake_minimum_required(VERSION 3.0.2)
project(march_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
    "../../../../ros2/src/shared/march_description/urdf/march3.xacro"
    "../../../../ros2/src/shared/march_description/urdf/march4.xacro"
    "../../../../ros2/src/shared/march_description/urdf/march4_with_koen_lengths.xacro"
    "../../../../ros2/src/shared/march_description/urdf/test_joint_rotational.xacro"
    "../../../../ros2/src/shared/march_description/urdf/test_joint_linear.xacro"
    "../../../../ros2/src/shared/march_description/urdf/test_joint_rotational_odrive.xacro"
)
build_and_install_xacro_files(${XACRO_FILES})

install(DIRECTORY "../../../../ros2/src/shared/march_description/urdf"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
