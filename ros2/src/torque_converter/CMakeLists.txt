cmake_minimum_required(VERSION 3.5)
project(torque_converter)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/torque_converter.hpp
  src/torque_converter.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "march_shared_msgs"
  "sensor_msgs"
)

add_executable(torque_converter_node
  src/torque_converter.cpp
  include/${PROJECT_NAME}/torque_converter.hpp
  src/torque_converter_node.cpp)
target_include_directories(torque_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  torque_converter_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "march_shared_msgs"
  "sensor_msgs"
)

install(TARGETS torque_converter_node
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    ament_add_gmock(${PROJECT_NAME}_test
    test/torque_converter_test.cpp
    test/test_runner.cpp
    # include
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>)
  
    # target_include_directories(${TEST_MY_COOL_PKG} PRIVATE "test/include" "include")
    ament_target_dependencies(
      ${PROJECT_NAME}_test 
      "rclcpp"
      "geometry_msgs"
      "std_msgs"
      "march_shared_msgs"
      "sensor_msgs")
  endif()

ament_package()
