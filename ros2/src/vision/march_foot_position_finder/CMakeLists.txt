cmake_minimum_required(VERSION 3.5)
project(march_foot_position_finder)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-w)

find_package(realsense2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
    src/preprocessor.cpp
    src/point_finder.cpp
    src/foot_position_finder.cpp
)

ament_target_dependencies(${PROJECT_NAME} tf2 tf2_ros rclcpp sensor_msgs std_msgs march_shared_msgs realsense2 pcl_conversions geometry_msgs visualization_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


add_executable(${PROJECT_NAME}_node src/foot_position_finder_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node tf2 tf2_ros rclcpp sensor_msgs std_msgs march_shared_msgs realsense2 pcl_conversions geometry_msgs visualization_msgs)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
