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
# set(cln "${PROJECT_SOURCE_DIR}/../libraries/ginac/cln/include")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
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
include_directories(include)

include_directories(${GINAC_BUILD_DIR})
include_directories(${cln})

add_library(${PROJECT_NAME} STATIC
  src/robot_description_node.cpp
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
  march_shared_msgs
)

# Install targets and executables
install(TARGETS
  robot_description_node
  state_estimator_node
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
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(robot_description_node_test test/robot_description_node_test.cpp)
  target_include_directories(robot_description_node_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(robot_description_node_test
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

  target_link_libraries(robot_description_node_test
    ${PROJECT_NAME}
    ${GINAC_LIB}
  )
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
