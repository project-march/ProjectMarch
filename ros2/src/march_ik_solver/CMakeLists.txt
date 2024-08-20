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
  add_compile_options(-Wall -Wextra -Wpedantic -fno-math-errno -ffast-math -O3 -lblas -fopenmp -DNDEBUG -march=native -mfma)
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
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(message_filters REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/ik_solver_node.cpp
  src/ik_solver.cpp
  src/task.cpp
  src/motion_task.cpp
  src/posture_task.cpp
  src/stability_task.cpp
  src/tilt_task.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  march_shared_msgs
  pinocchio
  ament_index_cpp
  message_filters
)

add_executable(ik_solver_node 
  src/ik_solver_node.cpp 
  src/ik_solver.cpp 
  src/task.cpp
  src/motion_task.cpp
  src/posture_task.cpp
  src/stability_task.cpp
  src/tilt_task.cpp
)

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
  pinocchio
  ament_index_cpp
  message_filters
)

target_link_libraries(ik_solver_node
  ${PROJECT_NAME}
  ${OpenMP_CXX_LIBRARIES}
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS
  ik_solver_node
  DESTINATION lib/${PROJECT_NAME})

# Add ik_manager_node executable
add_executable(ik_manager_node src/ik_manager_node.cpp)

target_include_directories(ik_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ik_manager_node
  rclcpp
  march_shared_msgs
)

install(TARGETS
  ik_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files.
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Install launch files.
install(DIRECTORY
  launch
  config
  plotjuggler
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  ament_add_gmock(${PROJECT_NAME}_test
    test/test_runner.cpp
    # test/ik_solver_test.cpp
    # src/ik_solver.cpp
    # test/task_test.cpp 
    # src/task.cpp
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

  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    ${OpenMP_CXX_LIBRARIES}
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
