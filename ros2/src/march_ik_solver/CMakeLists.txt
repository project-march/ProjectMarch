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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(urdf REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME} STATIC
  include/${PROJECT_NAME}/task.hpp
  src/task.cpp
  include/${PROJECT_NAME}/ik_solver.hpp
  src/ik_solver.cpp
  include/${PROJECT_NAME}/ik_solver_node.hpp
  src/ik_solver_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
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
  rclcpp_action
  rclcpp_components
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
)

add_executable(ik_solver_buffer_node src/ik_solver_buffer_node.cpp)
target_include_directories(ik_solver_buffer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ik_solver_buffer_node
  rclcpp
  Eigen3
  std_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
)

add_executable(current_joint_positions_server src/current_joint_positions_server.cpp)
ament_target_dependencies(current_joint_positions_server 
  rclcpp
  march_shared_msgs
  Eigen3)

add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server 
  rclcpp
  march_shared_msgs
  Eigen3)

add_executable(task_client src/task_client.cpp)
ament_target_dependencies(task_client 
  rclcpp
  march_shared_msgs
  Eigen3)

install(TARGETS
  ik_solver_node
  ik_solver_buffer_node
  current_joint_positions_server
  task_server
  task_client
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
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
