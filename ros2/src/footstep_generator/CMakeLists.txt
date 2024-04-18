cmake_minimum_required(VERSION 3.5)
project(footstep_generator)

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
find_package(march_description REQUIRED)
find_package(march_shared_msgs REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
  STATIC

  src/footstep_generator.cpp
  src/footstep_generator_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
)

# add_executable(footstep_generator_node src/footstep_generator_m9_node.cpp src/footstep_generator_m9.cpp)
add_executable(footstep_generator_node src/footstep_generator_node.cpp src/footstep_generator.cpp)
target_include_directories(footstep_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  footstep_generator_node
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
)

install(TARGETS footstep_generator_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

  

# if(BUILD_TESTING)
#   ament_add_gmock(${PROJECT_NAME}_test
#   test/footstep_gen_test.cpp
#   test/test_runner.cpp
#   # include
#   )
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include>)
#   ament_target_dependencies(
#     ${PROJECT_NAME}_test 
#     "rclcpp"
#     "geometry_msgs" 
#     "march_shared_msgs")
# endif()

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
