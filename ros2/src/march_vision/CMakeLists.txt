cmake_minimum_required(VERSION 3.5)
project(march_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(kindr REQUIRED) 
find_package(kindr_ros REQUIRED)  # SivertHavso/kindr_ros.git -b ros2, because the original package is not available for ROS2
find_package(realsense2 REQUIRED)
find_package(CGAL REQUIRED)  # -b 5.6.x, because 6.x+ requires Boost 1.72+, which is not available for Ubuntu 20.4
find_package(grid_map REQUIRED)  # -b foxy-devel
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS)
# find_package(OpenCV REQUIRED)

# Set build type to Release for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Create a static library for shared code
add_library(${PROJECT_NAME} STATIC
  src/elevation_mapping/elevation_mapping.cpp
  src/elevation_mapping/elevation_map.cpp
  src/elevation_mapping/point_XYZ_RGB_confidence_ratio.cpp 
  src/elevation_mapping/robot_motion_map_updater.cpp
  src/elevation_mapping/input_sources/input_source_manager.cpp
  src/elevation_mapping/input_sources/input.cpp
  src/elevation_mapping/postprocessing/postprocessing_pipeline_functor.cpp
  src/elevation_mapping/postprocessing/postprocessor_pool.cpp 
  src/elevation_mapping/sensor_processors/perfect_sensor_processor.cpp
  src/elevation_mapping/sensor_processors/sensor_processor_base.cpp 
  src/elevation_mapping/sensor_processors/structured_light_sensor_processor.cpp

  src/plane_segmentation/contour_segmentation.cpp
  src/plane_segmentation/planar_image_segmentation.cpp
  src/plane_segmentation/planar_region.cpp
  src/plane_segmentation/plane_segmentation_pipeline.cpp
  src/plane_segmentation/polygon_decomposition.cpp
  src/plane_segmentation/ransac_segmentation.cpp
  src/plane_segmentation/visualization_utils.cpp

  src/processing/camera_interface.cpp
  src/processing/point_cloud_registration.cpp
)

# Include directories for the library
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${grid_map_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
    ${kindr_ros_INCLUDE_DIRS}
    ${grid_map_msgs_INCLUDE_DIRS}  
    ${grid_map_core_INCLUDE_DIRS}
    ${grid_map_ros_INCLUDE_DIRS}
)

# Link libraries for the static library
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${grid_map_LIBRARIES}
  ${kindr_LIBRARIES}
  ${kindr_ros_LIBRARIES}
)

# Create executables for each node
add_executable(computer_vision_node src/computer_vision_node.cpp)
add_executable(elevation_mapping_node src/elevation_mapping/elevation_mapping_node.cpp)
add_executable(plane_segmentation_pipeline_node src/plane_segmentation/plane_segmentation_pipeline_node.cpp)

# Include directories for each executable
target_include_directories(computer_vision_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(elevation_mapping_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(plane_segmentation_pipeline_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link dependencies for each executable
ament_target_dependencies(computer_vision_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  grid_map_msgs  
  grid_map_core
  grid_map_ros
  realsense2
  kindr
  kindr_ros
)

ament_target_dependencies(elevation_mapping_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  grid_map_msgs  
  grid_map_core
  grid_map_ros
  realsense2
  kindr
  kindr_ros
)

ament_target_dependencies(plane_segmentation_pipeline_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  grid_map_msgs  
  grid_map_core
  grid_map_ros
  realsense2
  kindr
  kindr_ros
)

# Link the static library to each executable
target_link_libraries(computer_vision_node
  ${PROJECT_NAME}
)

target_link_libraries(elevation_mapping_node
  ${PROJECT_NAME}
)

target_link_libraries(plane_segmentation_pipeline_node
  ${PROJECT_NAME}
)

# Install targets and executables
install(TARGETS
  computer_vision_node
  elevation_mapping_node
  plane_segmentation_pipeline_node
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
