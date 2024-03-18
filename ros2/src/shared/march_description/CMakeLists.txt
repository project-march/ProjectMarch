cmake_minimum_required(VERSION 3.5)
project(march_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

set(XACRO_FILES
        urdf/test_xacros/test_joint_linear.xacro
        urdf/test_xacros/test_joint_rotational.xacro
        urdf/march3/march3.xacro
        urdf/march4/march4.xacro
        urdf/march4/march4_with_koen_lengths.xacro
        urdf/march6/march6.xacro
        urdf/march6/march6_three_cameras.xacro
        urdf/march7/march7.xacro
        urdf/march8.xacro
)

# Without this the package link of stl files in gazebo doesn't work
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

build_and_install_xacro_files(${XACRO_FILES})
