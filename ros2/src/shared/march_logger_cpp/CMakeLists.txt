cmake_minimum_required(VERSION 3.5)
project(march_logger_cpp)

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

add_library(
        ${PROJECT_NAME}
        SHARED
        src/ros_logger.cpp
)

ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# These must be above the two installs otherwise can't be recognized by other packages.
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)  # For the ros_logger
# INSTALL
install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()
