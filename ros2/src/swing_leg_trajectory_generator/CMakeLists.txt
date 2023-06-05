cmake_minimum_required(VERSION 3.5)
project(swing_leg_trajectory_generator)

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
find_package(march_shared_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
        STATIC
        include/${PROJECT_NAME}/swing_leg_trajectory_generator.hpp
        include/${PROJECT_NAME}/swing_leg_trajectory_generator_node.hpp
        src/swing_leg_trajectory_generator.cpp
        src/swing_leg_trajectory_generator_node.cpp
        )

ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
        "std_msgs"
        "nav_msgs"
)

add_executable(swing_leg_trajectory_generator_node src/swing_leg_trajectory_generator_node.cpp src/swing_leg_trajectory_generator.cpp
        include/${PROJECT_NAME}/swing_leg_trajectory_generator_node.hpp include/${PROJECT_NAME}/swing_leg_trajectory_generator.hpp)
target_include_directories(swing_leg_trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        swing_leg_trajectory_generator_node
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
        "std_msgs"
        "nav_msgs"
)

# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

install(TARGETS swing_leg_trajectory_generator_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gmock(${PROJECT_NAME}_test
          test/swing_leg_trajectory_generator_test.cpp
          test/test_runner.cpp
          # include
          )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
          $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
          ${PROJECT_NAME}_test
          "rclcpp"
          "march_shared_msgs"
          "geometry_msgs"
          "nav_msgs")
endif()

ament_package()
