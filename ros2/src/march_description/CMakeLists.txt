cmake_minimum_required(VERSION 3.5)
project(march_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(catkin REQUIRED COMPONENTS xacro)
find_package(xacro REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
    urdf/march3.xacro
    urdf/march4.xacro
    urdf/test_joint_linear.xacro
    urdf/test_joint_rotational.xacro
)

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

build_and_install_xacro_files(${XACRO_FILES})



