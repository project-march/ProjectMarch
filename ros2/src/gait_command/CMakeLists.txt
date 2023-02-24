cmake_minimum_required(VERSION 3.5)
project(gait_command)

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
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
        STATIC
        include/${PROJECT_NAME}/gait_command.hpp
        include/${PROJECT_NAME}/gait_command_node.hpp
        src/gait_command.cpp
        src/gait_command_node.cpp
)

ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "march_shared_msgs"
)

add_executable(gait_command_node src/gait_command_node.cpp)
ament_target_dependencies(gait_command_node rclcpp march_shared_msgs)
target_include_directories(gait_command_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS gait_command_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
endif()

# EXPORT
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
        "rclcpp"
        "march_shared_msgs"
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

