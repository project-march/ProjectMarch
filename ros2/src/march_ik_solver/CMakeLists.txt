cmake_minimum_required(VERSION 3.5)
project(march_ik_solver)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/task.cpp
  src/ik_solver.cpp
  src/ik_solver_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
)

add_executable(ik_solver_node src/ik_solver_node.cpp src/ik_solver.cpp src/task.cpp)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ik_solver_node
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
)

add_executable(ik_solver_point_evaluator_node src/ik_solver_point_evaluator_node.cpp)
target_include_directories(ik_solver_point_evaluator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ik_solver_point_evaluator_node
  rclcpp
  Eigen3
  std_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
)

install(TARGETS
  ik_solver_node
  ik_solver_point_evaluator_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Install launch files.
install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  ament_add_gmock(${PROJECT_NAME}_test
    src/task.cpp
    test/task_test.cpp 
    test/test_runner.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    Eigen3
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    march_shared_msgs
  )

  ament_export_dependencies(
    rclcpp
    std_msgs
    Eigen3
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    march_shared_msgs
  )

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
