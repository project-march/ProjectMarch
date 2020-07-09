cmake_minimum_required(VERSION 3.0.2)
project(march_joint_inertia_controller)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    controller_manager
    controller_interface
    joint_state_controller
    joint_trajectory_controller
    pluginlib
    roscpp
    urdf
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    pluginlib
    roscpp
)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/inertia_estimator.h
    include/${PROJECT_NAME}/joint_trajectory_inertia_controller.h
    src/inertia_estimator.cpp
    src/joint_inertia_controller.cpp
    src/joint_trajectory_inertia_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

install(FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
