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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(realsense2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS)

# Correct path to libraries
set(LIBRARIES_DIR "${PROJECT_SOURCE_DIR}/../libraries")
add_subdirectory("${LIBRARIES_DIR}/grid_map/grid_map" grid_map)
add_subdirectory("${LIBRARIES_DIR}/CGAL" CGAL)
add_subdirectory("${LIBRARIES_DIR}/kindr" kindr)
add_subdirectory("${LIBRARIES_DIR}/kindr_ros/kindr_ros" kindr_ros)

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

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${LIBRARIES_DIR}/grid_map
    ${LIBRARIES_DIR}/kindr/include
    ${LIBRARIES_DIR}/kindr_ros/kindr_ros/include
    ${LIBRARIES_DIR}/CGAL
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  grid_map_msgs
  grid_map_core
  grid_map_ros
  realsense2
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

add_executable(computer_vision_node
  src/computer_vision_node.cpp
  src/processing/camera_interface.cpp
  src/processing/point_cloud_registration.cpp
)

target_include_directories(computer_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(computer_vision_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  grid_map_msgs
  grid_map_core
  grid_map_ros
  realsense2
)

target_link_libraries(computer_vision_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

add_executable(elevation_mapping_node
  src/elevation_mapping/elevation_mapping_node.cpp
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
)

target_include_directories(elevation_mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/elevation_mapping>
  $<INSTALL_INTERFACE:include/elevation_mapping>
  ${LIBRARIES_DIR}/grid_map
  ${LIBRARIES_DIR}/kindr/include
  ${LIBRARIES_DIR}/kindr_ros/kindr_ros/include
  ${LIBRARIES_DIR}/CGAL

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
  tf2_eigen
  pcl_conversions
  grid_map_msgs
  grid_map_core
  grid_map_ros
  realsense2
)

target_link_libraries(elevation_mapping_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

add_executable(plane_segmentation_pipeline_node
  src/plane_segmentation/plane_segmentation_pipeline_node.cpp
  src/plane_segmentation/contour_segmentation.cpp
  src/plane_segmentation/planar_image_segmentation.cpp
  src/plane_segmentation/planar_region.cpp
  src/plane_segmentation/plane_segmentation_pipeline.cpp
  src/plane_segmentation/polygon_decomposition.cpp
  src/plane_segmentation/ransac_segmentation.cpp
  src/plane_segmentation/visualization_utils.cpp
)

target_include_directories(plane_segmentation_pipeline_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/plane_segmentation>
  $<INSTALL_INTERFACE:include/plane_segmentation>
  ${LIBRARIES_DIR}/grid_map
  ${LIBRARIES_DIR}/kindr/include
  ${LIBRARIES_DIR}/kindr_ros/kindr_ros/include
  ${LIBRARIES_DIR}/CGAL

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
  tf2_eigen
  pcl_conversions
  grid_map_msgs
  grid_map_core
  grid_map_ros
  realsense2
)

target_link_libraries(plane_segmentation_pipeline_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  computer_vision_node
  elevation_mapping_node
  plane_segmentation_pipeline_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
