cmake_minimum_required(VERSION 3.5)
project(ik_solver)

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
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_description REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/ik_solver.hpp
  src/ik_solver.cpp
  include/${PROJECT_NAME}/ik_solver_node.hpp
  src/ik_solver_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
  pinocchio
  urdf
  Eigen3
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ik_solver_node 
src/ik_solver_node.cpp 
src/ik_solver.cpp 
include/${PROJECT_NAME}/ik_solver.hpp
include/${PROJECT_NAME}/ik_solver_node.hpp)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_link_libraries (ik_solver_node Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(
  ik_solver_node
  pinocchio
  Eigen3
  urdf
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
)

install(TARGETS ik_solver_node
EXPORT export_${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

#Export
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  ament_add_gmock(${PROJECT_NAME}_test
  test/test_runner.cpp
  # include
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test 
    pinocchio
    Eigen3
    urdf
    "rclcpp"
    "geometry_msgs" 
    "march_shared_msgs")
endif()

ament_package()
