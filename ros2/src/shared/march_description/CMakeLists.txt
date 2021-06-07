cmake_minimum_required(VERSION 3.5)
project(march_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
        )

set(XACRO_FILES
    urdf/march3.xacro
    urdf/march4.xacro
    urdf/march6.xacro
    urdf/march4_with_koen_lengths.xacro
    urdf/test_joint_linear.xacro
    urdf/test_joint_rotational.xacro
    urdf/test_joint_linear_odrive.xacro
    urdf/test_joint_rotational_odrive.xacro
    urdf/test_right_leg_rotational.xacro
    urdf/test_left_leg_rotational.xacro
    urdf/test_left_odrive_upper.xacro
    urdf/test_left_odrive_lower.xacro
    urdf/march6_no_translational_joints.xacro
)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

build_and_install_xacro_files(${XACRO_FILES})
