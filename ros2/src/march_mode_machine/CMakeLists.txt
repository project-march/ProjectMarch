cmake_minimum_required(VERSION 3.5)
project(march_mode_machine)

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

# Run the Python script to generate exo_mode.hpp
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(PYTHON_SCRIPT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/generate/generate_exo_mode_class.py")

add_custom_target(
    run_python_script ALL
    COMMAND ${Python3_EXECUTABLE} ${PYTHON_SCRIPT_PATH}
    COMMENT "Generating ezo_mode.hpp from JSON file"
    VERBATIM
)

add_library(${PROJECT_NAME}
  STATIC
  src/mode_machine.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "march_shared_msgs"
  "mujoco_interfaces"
)

add_executable(mode_machine_node 
  src/mode_machine_node.cpp 
  src/mode_machine.cpp)

add_executable(test_joints_mode_machine_node
  src/test_joints_mode_machine_node.cpp
  src/test_joints_mode_machine.cpp
)

add_executable(mode_machine_cartesian_node
  src/mode_machine_cartesian_node.cpp 
  src/mode_machine_cartesian.cpp)

# Link the mode_machine library to the test_joints_mode_machine_node executable
target_link_libraries(test_joints_mode_machine_node
  ${PROJECT_NAME}
)

target_link_libraries(mode_machine_cartesian_node
  ${PROJECT_NAME}
)

# Link against ROS 2 dependencies
ament_target_dependencies(test_joints_mode_machine_node
  rclcpp
  march_shared_msgs
  )

ament_target_dependencies(mode_machine_cartesian_node
  rclcpp
  march_shared_msgs
  )

target_include_directories(mode_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        mode_machine_node
        "rclcpp"
        "march_shared_msgs"
        "mujoco_interfaces"
)

install(TARGETS 
  mode_machine_node
  test_joints_mode_machine_node
  mode_machine_cartesian_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

# Install 
install(DIRECTORY
generate
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  # add_launch_test(test/launch_tests/invalid_transition_pub_sub_test.py )
#  add_launch_test(test/launch_tests/valid_transition_pub_sub_test.py)
  ament_add_gmock(${PROJECT_NAME}_test
    test/mode_machine_test.cpp
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
