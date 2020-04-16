cmake_minimum_required(VERSION 3.10.2)
project(march_gazebo_plugins LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    march_shared_resources
    roscpp
)
find_package(gazebo REQUIRED)

include_directories(
    include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS march_shared_resources
)

add_library(${PROJECT_NAME}
    src/com_controller_plugin.cpp
    src/obstacle_controller.cpp
    src/stairs_controller.cpp
    src/walk_controller.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
