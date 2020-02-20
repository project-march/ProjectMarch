cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(march_gazebo_plugins)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
        march_shared_resources
        )
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/com_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
