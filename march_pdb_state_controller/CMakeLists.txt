cmake_minimum_required(VERSION 2.8.3)
project(march_pdb_state_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    march_hardware
    march_hardware_interface
    march_shared_resources
    pluginlib
    realtime_tools
    roscpp
    roslint
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    hardware_interface
    march_hardware
    march_hardware_interface
    march_shared_resources
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "*.cpp"
    "*.h"
)

roslint_cpp(${lintfiles})

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/march_pdb_state_controller.h
    src/march_pdb_state_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES march_pdb_state_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
