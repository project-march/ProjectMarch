cmake_minimum_required(VERSION 3.16)

project(march_controller_template DESCRIPTION "Template"
    LANGUAGES CXX C)

# Enable the FetchContent CMake module.
# Needed to fetch online content (from acado git repo) and allow cmake to use it
include(FetchContent)

# Define which packages this package depends on
find_package(catkin REQUIRED 
        COMPONENTS
        control_msgs
        controller_interface
        controller_manager
        joint_state_controller
        joint_trajectory_controller
        march_shared_msgs
        pluginlib
        roscpp
        std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    # Define the catkin projects that this project depends on. If another project
    # does "find_package(march_acado_mpc)" these packages will be seen as
    # transitive dependencies.
    CATKIN_DEPENDS
    control_msgs
    controller_interface
    march_shared_msgs
    pluginlib
    roscpp
    std_msgs
)

include_directories(
        include
        "${catkin_INCLUDE_DIRS}"
)

add_library(${PROJECT_NAME}
        src/joint_trajectory_template.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
