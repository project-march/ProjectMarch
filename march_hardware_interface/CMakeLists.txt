cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager
  march_hardware
)

catkin_package(
        CATKIN_DEPENDS
)



include_directories(include ${catkin_INCLUDE_DIRS})

set(SOURCES src/march_hardware_interface.cpp src/march_hardware_interface_node.cpp)

add_executable(march_hardware_interface_node ${SOURCES} ${SOURCE})

add_dependencies(march_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(march_hardware_interface_node ${catkin_LIBRARIES})

# Give sudo access to ethernet port
add_custom_command(
        TARGET march_hardware_interface_node
        POST_BUILD
        COMMAND sudo setcap cap_net_raw+ep march_hardware_interface_node
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/march_hardware_interface
)