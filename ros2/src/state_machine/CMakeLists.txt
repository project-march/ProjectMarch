cmake_minimum_required(VERSION 3.5)
project(state_machine)

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
find_package(mujoco_interfaces REQUIRED)
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/state_machine.hpp
  include/${PROJECT_NAME}/state_machine_node.hpp
  src/state_machine_node.cpp
  src/state_machine.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "march_shared_msgs"
  "mujoco_interfaces"
)

add_executable(state_machine_node src/state_machine_node.cpp src/state_machine.cpp include/${PROJECT_NAME}/state_machine.hpp
        include/${PROJECT_NAME}/state_machine_node.hpp)
target_include_directories(state_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        state_machine_node
        "rclcpp"
        "march_shared_msgs"
        "mujoco_interfaces"
)

install(TARGETS state_machine_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch_tests/invalid_transition_pub_sub_test.py )
  add_launch_test(test/launch_tests/valid_transition_pub_sub_test.py)
  ament_add_gmock(${PROJECT_NAME}_test
    test/state_machine_test.cpp
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
    "mujoco_interfaces")
endif()

# EXPORT
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
        "rclcpp"
        "march_shared_msgs"
        "mujoco_interfaces"
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
