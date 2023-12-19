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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(march_description REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(urdf REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/march_state_estimator_node.cpp
  src/urdf_parser.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  march_description
  urdf
)

add_executable(march_state_estimator_node
  src/march_state_estimator_node.cpp
  src/urdf_parser.cpp
  src/robot_description.cpp
)
target_include_directories(march_state_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(march_state_estimator_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  march_description
  urdf
)

install(TARGETS
  march_state_estimator_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

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
