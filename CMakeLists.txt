cmake_minimum_required(VERSION 2.8.3)
project(march_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
)

set(THIS_PACKAGE_INCLUDE_DIRS
    doc/development/cpp_ros_unit_test_tutorial/include
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Add tutorials with a CMakeLists.txt here
add_subdirectory(doc/development/cpp_ros_unit_test_tutorial)
