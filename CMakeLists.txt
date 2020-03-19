cmake_minimum_required(VERSION 2.8.3)
project(march_tutorials)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_python_setup()

catkin_package()

set(THIS_PACKAGE_INCLUDE_DIRS
    doc/development/ros_test_tutorial/include
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Add tutorials with a CMakeLists.txt here
add_subdirectory(doc/development/ros_test_tutorial)

install(DIRECTORY doc/useful_ros_tools/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
