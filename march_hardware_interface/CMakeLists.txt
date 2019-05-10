cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  controller_manager
  march_hardware
  march_hardware_builder
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
        INCLUDE_DIRS
        include
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})


include_directories(include ${catkin_INCLUDE_DIRS})

set(SOURCES src/march_hardware_interface.cpp src/march_hardware_interface_node.cpp)

add_executable(march_hardware_interface_node ${SOURCES} ${SOURCE})

add_dependencies(march_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(march_hardware_interface_node ${catkin_LIBRARIES})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

# Give sudo access to ethernet port
add_custom_command(
        TARGET march_hardware_interface_node
        COMMAND sudo setcap cap_net_raw+ep march_hardware_interface_node
        WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
        POST_BUILD
)
