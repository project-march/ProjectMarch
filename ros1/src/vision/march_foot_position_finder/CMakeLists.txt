cmake_minimum_required(VERSION 3.5)
project(march_foot_position_finder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

add_definitions(-w)

# find dependencies
find_package(realsense2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros
    march_shared_msgs
    dynamic_reconfigure
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

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

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/preprocessor.cpp
    src/point_finder.cpp
    src/foot_position_finder.cpp
)

add_executable(foot_position_finder_node src/foot_position_finder_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(foot_position_finder_node ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Mark executables for installation
install(TARGETS foot_position_finder_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
