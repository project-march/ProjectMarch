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

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_package()