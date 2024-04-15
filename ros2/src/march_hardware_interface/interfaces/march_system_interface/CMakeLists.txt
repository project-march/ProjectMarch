cmake_minimum_required(VERSION 3.5)
project(march_system_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(march_hardware REQUIRED)
find_package(march_hardware_builder REQUIRED)
find_package(march_logger_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_utility REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/march_mock_system_interface.cpp
        include/march_system_interface/march_mock_system_interface.hpp
        src/march_exo_system_interface.cpp
        include/march_system_interface/hwi_util.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        ${PROJECT_NAME}
        hardware_interface
        pluginlib
        rclcpp
        march_hardware
        march_hardware_builder
        march_logger_cpp
        yaml-cpp
        march_utility
        sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface march_system_interface.xml)

# EXPORT
# These must be above the two installs otherwise can't be recognized by other packages.
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
        march_hardware
        march_hardware_builder
        march_logger_cpp
        yaml-cpp
        march_utility
        sensor_msg
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
