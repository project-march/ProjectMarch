cmake_minimum_required(VERSION 3.5)
project(march_gait_planning)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest)

# Include directories
include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/gait_planning_joint_angles_node.cpp
  src/gait_planning_joint_angles.cpp
  src/gait_planning_node.cpp
  src/gait_planning.cpp
  src/test_joints_gait_planning_node.cpp
  src/test_setup_gait_planning_node.cpp
  src/test_setup_gait_planning.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  march_shared_msgs
  geometry_msgs
  trajectory_msgs
  ament_index_cpp
  std_msgs
)

# Create the gait_planning_node executable
add_executable(gait_planning_node
  src/gait_planning_node.cpp
  src/gait_planning.cpp
)
add_executable(gait_planning_angles_node
  src/gait_planning_joint_angles_node.cpp
  src/gait_planning_joint_angles.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(gait_planning_node
  rclcpp
  march_shared_msgs
  geometry_msgs
  ament_index_cpp
  std_msgs
  # filesystem
  )

ament_target_dependencies(gait_planning_angles_node
  rclcpp
  trajectory_msgs
  march_shared_msgs
  ament_index_cpp
  std_msgs
  )

# Install the executable
install(TARGETS gait_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gait_planning_angles_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create the test_gait_planning executable in the test_setup_gait_planning folder
add_executable(test_setup_gait_planning_node
  src/test_setup_gait_planning_node.cpp
  src/test_setup_gait_planning.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(test_setup_gait_planning_node
  rclcpp
  march_shared_msgs
  geometry_msgs
  ament_index_cpp
  # filesystem
)

# Install the test_gait_planning_node executable
install(TARGETS test_setup_gait_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create the test_gait_planning executable in the test_setup_gait_planning folder
add_executable(test_joints_gait_planning_node
  src/test_joints_gait_planning_node.cpp
  src/test_setup_gait_planning.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(test_joints_gait_planning_node
  rclcpp
  march_shared_msgs
  geometry_msgs
  ament_index_cpp
  # filesystem
)

# Install the test_gait_planning_node executable
install(TARGETS test_joints_gait_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY 
  launch test m9_gait_files
  DESTINATION share/${PROJECT_NAME}/ 
)

if(BUILD_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/gait_planning_test.cpp 
    src/gait_planning.cpp
    test/test_setup_gait_planning_test.cpp
    src/test_setup_gait_planning.cpp
    test/all_gait_planning_test_runner.cpp
    )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
    ament_index_cpp
  )
  if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES ${PROJECT_NAME}_test
        )
  endif()
endif()

# Add the necessary Python modules
ament_package()