cmake_minimum_required(VERSION 3.5.2)
project(march_realsense_reader)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros
    march_shared_msgs
    dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
    config/pointcloud_parameters.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros
    march_shared_msgs
    dynamic_reconfigure
)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/realsense_reader.cpp
    src/pointcloud_processor/preprocessor.cpp
    src/pointcloud_processor/region_creator.cpp
    src/pointcloud_processor/hull_finder.cpp
    src/pointcloud_processor/parameter_determiner.cpp
)

add_executable(${PROJECT_NAME}_node src/realsense_reader_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############
## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ include/pointcloud_processor/
        include/utilities/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
