cmake_minimum_required(VERSION 3.5)
project(march_vision)

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

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(realsense2 REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(CGAL REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} STATIC

    src/computer_vision_node.cpp
    src/computer_vision.cpp
    # src/march_octree/march_octree_node.cpp
    # src/march_octree/march_octree.cpp 
    # src/march_octree/march_octree_utils.cpp 
    # src/plane_segmentation/march_polygon.cpp 
    # src/plane_segmentation/polygon_calculator.cpp 
    #src/processing/cameras_interface.cpp
    #src/processing/point_cloud_registration.cpp
    #src/processing/input_source_manager.cpp

)

ament_target_dependencies(${PROJECT_NAME}

    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    march_shared_msgs
    Eigen3
    tf2_ros
    tf2
    tf2_geometry_msgs

    pcl_conversions
)

add_executable(computer_vision_node

    src/computer_vision_node.cpp
    src/computer_vision.cpp
    # src/march_octree/march_octree_node.cpp
    # src/march_octree/march_octree.cpp 
    # src/march_octree/march_octree_utils.cpp 
    # src/plane_segmentation/march_polygon.cpp 
    # src/plane_segmentation/polygon_calculator.cpp
    #src/processing/cameras_interface.cpp
    #src/processing/point_cloud_registration.cpp
    #src/processing/input_source_manager.cpp
)

target_include_directories(computer_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(computer_vision_node
    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    march_shared_msgs
    Eigen3
    realsense2
    tf2
    tf2_ros
    tf2_geometry_msgs

    pcl_conversions
    
)

target_link_libraries(computer_vision_node
  ${PROJECT_NAME}
  ${realsense2_LIBRARY}
  ${PCL_LIBRARIES}
  #${EIGEN3_INCLUDE_DIRS}
  #${CGAL_LIBRARIES}
)

# Install targets and executables
install(TARGETS
  computer_vision_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)