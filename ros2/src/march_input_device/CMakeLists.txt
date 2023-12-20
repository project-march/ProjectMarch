cmake_minimum_required(VERSION 3.5)
project(march_input_device)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)

# Include directories
include_directories(include)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Create the executable
add_executable(input_device_node
  src/input_device_node.cpp
  src/input_device.cpp
)

# Create a library from input_device.cpp
add_library(input_device_lib src/input_device.cpp)

# Link against ROS 2 dependencies
ament_target_dependencies(input_device_node
  rclcpp
  march_shared_msgs
  )

target_link_libraries(input_device_node ${CURSES_LIBRARIES})

# Install the executable
install(TARGETS input_device_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/input_device_test.cpp)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
  )

  target_link_libraries(${PROJECT_NAME}_test input_device_lib)
endif()


# Add the necessary Python modules
ament_package()
