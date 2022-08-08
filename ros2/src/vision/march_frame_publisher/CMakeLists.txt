cmake_minimum_required(VERSION 3.5)
project(march_frame_publisher)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(march_shared_msgs REQUIRED)

add_library(${PROJECT_NAME}
    src/point_cloud_aligner.cpp
    src/frame_publisher.cpp
)

add_executable(${PROJECT_NAME}_node src/frame_publisher_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node tf2 tf2_ros rclcpp geometry_msgs std_srvs rcl_interfaces march_shared_msgs)

ament_target_dependencies(${PROJECT_NAME} tf2 tf2_ros rclcpp geometry_msgs std_srvs rcl_interfaces march_shared_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_node yaml-cpp)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

