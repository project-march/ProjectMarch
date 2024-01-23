cmake_minimum_required(VERSION 3.5)
project(march_state_estimator)

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

# Set external library directories
set(GINAC_BUILD_DIR "${PROJECT_SOURCE_DIR}/../libraries/ginac/build")
set(GINAC_LIB "${PROJECT_SOURCE_DIR}/../libraries/ginac/build/ginac/libginac.so")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_description REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
include_directories(include)

# include_directories(${GINAC_BUILD_DIR})

add_library(${PROJECT_NAME} STATIC
  src/robot_description_node.cpp
  src/robot_description.cpp
  src/robot_node.cpp
  src/robot_mass.cpp
  src/robot_joint.cpp
  src/sensor_fusion_node.cpp
  src/state_estimator_node.cpp
  src/transform_publisher_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_description
  march_shared_msgs
  urdf
  Eigen3
)

# Create robot_description_node executable
add_executable(robot_description_node
  src/robot_description_node.cpp
  src/robot_description.cpp
  src/robot_node.cpp
  src/robot_mass.cpp
  src/robot_joint.cpp
)

target_include_directories(robot_description_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(robot_description_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_description
  march_shared_msgs
  urdf
  Eigen3
)

target_link_libraries(robot_description_node
  ${PROJECT_NAME}
  ${GINAC_LIB}
)

# Create state_estimator_node executable
add_executable(state_estimator_node
  src/robot_description_node.cpp
  src/robot_description.cpp
  src/robot_node.cpp
  src/robot_mass.cpp
  src/robot_joint.cpp
  src/sensor_fusion_node.cpp
  src/state_estimator_node.cpp
)
target_include_directories(state_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(state_estimator_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_description
  march_shared_msgs
  urdf
  Eigen3
)

target_link_libraries(state_estimator_node
  ${PROJECT_NAME}
  ${GINAC_LIB}
)

# Create transform broadcaster executable
add_executable(transform_publisher_node
  src/transform_publisher_node.cpp
)

target_include_directories(transform_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(transform_publisher_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  march_shared_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets and executables
install(TARGETS
  robot_description_node
  state_estimator_node
  transform_publisher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)

  # find_package(launch_testing_ament_cmake)
  # file(COPY test/urdf/rotational_test_setup.urdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/urdf)

  ament_add_gmock(test_runner
    test/robot_description_test.cpp
    test/test_runner.cpp
  )

  target_include_directories(test_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(test_runner
    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    march_description
    march_shared_msgs
    urdf
    Eigen3

  )

  target_link_libraries(test_runner
    ${PROJECT_NAME}
    ${GINAC_LIB}
  )

  # # EXPORT
  # ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    march_description
    march_shared_msgs
    urdf
    Eigen3

  )
  
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
