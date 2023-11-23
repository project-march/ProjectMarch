cmake_minimum_required(VERSION 3.5)
project(march_input_device)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Create the executable
add_executable(input_device_node
  src/input_device_node.cpp
  src/input_device.cpp
)

# Link against ROS 2 dependencies
ament_target_dependencies(input_device_node
  rclcpp
)

# Install the executable
install(TARGETS input_device_node
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

# Add the necessary Python modules
ament_package()
