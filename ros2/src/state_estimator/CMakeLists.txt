cmake_minimum_required(VERSION 3.5)
project(state_estimator)

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
find_package(sensor_msgs REQUIRED)
find_package(march_description REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/state_estimator.hpp
  src/state_estimator.cpp
  include/${PROJECT_NAME}/joint_estimator.hpp
  src/joint_estimator.cpp
  include/${PROJECT_NAME}/com_estimator.hpp
  src/com_estimator.cpp
  include/${PROJECT_NAME}/cop_estimator.hpp
  src/cop_estimator.cpp
  include/${PROJECT_NAME}/zmp_estimator.hpp
  src/zmp_estimator.cpp
  include/${PROJECT_NAME}/imu_estimator.hpp
  src/imu_estimator.cpp
  include/${PROJECT_NAME}/footstep_estimator.hpp
  src/footstep_estimator.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "march_description"
  "march_shared_msgs"
  "nav_msgs"
  "visualization_msgs"
  "tf2_ros"
  "tf2"
  "tf2_kdl"
  "tf2_geometry_msgs"
)

add_executable(state_estimator_node 
  src/state_estimator_node.cpp 
  include/${PROJECT_NAME}/state_estimator.hpp
  src/state_estimator.cpp
  include/${PROJECT_NAME}/joint_estimator.hpp
  src/joint_estimator.cpp
  include/${PROJECT_NAME}/com_estimator.hpp
  src/com_estimator.cpp
  include/${PROJECT_NAME}/cop_estimator.hpp
  src/cop_estimator.cpp
  include/${PROJECT_NAME}/zmp_estimator.hpp
  src/zmp_estimator.cpp
  include/${PROJECT_NAME}/imu_estimator.hpp
  src/imu_estimator.cpp
  include/${PROJECT_NAME}/footstep_estimator.hpp
  src/footstep_estimator.cpp
  )
target_include_directories(state_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(
  state_estimator_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "march_description"
  "march_shared_msgs"
  "nav_msgs"
  "tf2_ros"
  "visualization_msgs"
  "tf2"
  "tf2_kdl"
  "tf2_geometry_msgs"
)

install(TARGETS state_estimator_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch_tests/state_estimator_com_estimator_test.py )
  add_launch_test(test/launch_tests/state_estimator_robot_feet_test.py )

  ament_add_gmock(${PROJECT_NAME}_test
  # test/joint_estimator_test.cpp
  test/cop_estimator_test.cpp
  test/com_estimator_test.cpp
  test/state_estimator_test.cpp
  test/test_runner.cpp
  # test/mocks/mock_state_estimator.hpp
  # include
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test
    "rclcpp"
    "geometry_msgs"
    "sensor_msgs"
    "march_shared_msgs"
    "nav_msgs"
    "tf2_ros"
    "tf2_geometry_msgs"
    "tf2"
    "tf2_kdl")
endif()


# EXPORT
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
        "rclcpp"
        "geometry_msgs"
        "sensor_msgs"
        "march_shared_msgs"
        "nav_msgs"
        "tf2_ros"
        "tf2_geometry_msgs"
        "tf2"
        "tf2_kdl"
)
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_package()
