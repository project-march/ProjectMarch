cmake_minimum_required(VERSION 3.5)
project(march_utility)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(march_description REQUIRED)

# Install cpp libraries
add_library(${PROJECT_NAME}
        src/node_utils.cpp
        )
ament_target_dependencies(${PROJECT_NAME} march_shared_msgs march_description rclcpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(TEST_NAME test)

    install(DIRECTORY test/resources
        DESTINATION share/${PROJECT_NAME}/test)
endif()

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
ament_package()
