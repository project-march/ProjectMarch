cmake_minimum_required(VERSION 3.5)
project(march_gait_planning)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Create the gait_planning_node executable
add_executable(gait_planning_node
  src/gait_planning_node.cpp
  src/gait_planning.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(gait_planning_node
  rclcpp
  march_shared_msgs
  geometry_msgs
)

# Install the gait_planning_node executable
install(TARGETS gait_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create the test_gait_planning executable in the test_setup_gait_selection folder
add_executable(test_gait_planning
  src/test_setup_gait_selection/test_gait_planning_node.cpp
  src/test_setup_gait_selection/test_gait_planning.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(test_gait_planning
  rclcpp
  march_shared_msgs
  geometry_msgs
)

# Install the test_gait_planning_node executable
install(TARGETS test_gait_planning_node
  DESTINATION lib/${PROJECT_NAME}/test_setup_gait_selection
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
  launch
  DESTINATION share/${PROJECT_NAME}/ 
)

# Add the necessary Python modules
ament_package()
