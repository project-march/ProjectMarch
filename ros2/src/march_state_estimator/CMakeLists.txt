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
  add_compile_options(-Wall -Wextra -Wpedantic -fno-math-errno -ffast-math -O3 -fopenmp -DNDEBUG)
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

find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.6)
include_directories(include)

# include_directories(${GINAC_BUILD_DIR})

add_library(${PROJECT_NAME} STATIC
  src/robot_description_node.cpp
  src/robot_description.cpp
  src/robot_node.cpp
  src/robot_mass.cpp
  src/robot_joint.cpp
  src/robot_zmp.cpp
  src/sensor_fusion_node.cpp
  src/sensor_fusion.cpp
  src/torque_converter.cpp
  src/state_estimator_node.cpp
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

# Create state_estimator_node executable
add_executable(state_estimator_node
  src/robot_description_node.cpp
  src/robot_description.cpp
  src/robot_node.cpp
  src/robot_mass.cpp
  src/robot_joint.cpp
  src/robot_zmp.cpp
  src/sensor_fusion_node.cpp
  src/sensor_fusion.cpp
  src/torque_converter.cpp
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
  tf2
  tf2_ros
)

target_link_libraries(state_estimator_node
  ${PROJECT_NAME}
  ${GINAC_LIB}
  ${YAMLCPP_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

# Install targets and executables
install(TARGETS
  state_estimator_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)

  # find_package(launch_testing_ament_cmake)
  # file(COPY test/urdf/rotational_test_setup.urdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/urdf)

  ament_add_gmock(test_runner
    test/test_runner.cpp
    test/robot_description_test.cpp
    src/robot_description.cpp
    test/torque_converter_test.cpp  
    src/torque_converter.cpp
    test/sensor_fusion_test.cpp
    src/sensor_fusion.cpp
    src/robot_node.cpp
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
    ${YAMLCPP_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
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
