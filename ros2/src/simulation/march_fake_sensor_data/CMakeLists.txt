cmake_minimum_required(VERSION 3.5)
project(march_fake_sensor_data LANGUAGES CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(march_utility REQUIRED)

# the library that contains the logic
add_library(${PROJECT_NAME}
    src/FakeTemperatureData.cpp
    src/UniformDistribution.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs std_msgs march_shared_msgs march_utility)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# the executable that is run in ROS and links with the library defined above
add_executable(${PROJECT_NAME}_node src/main.cpp    )
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs march_shared_msgs march_utility)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Add a gtest executables
    ament_add_gtest(${PROJECT_NAME}-uniform-distribution-test
        test/UniformDistributionTest.cpp
    )
    target_link_libraries(${PROJECT_NAME}-uniform-distribution-test ${PROJECT_NAME})
    ament_add_gtest(${PROJECT_NAME}-fake-temperature-data-test
        test/FakeTemperatureDataTest.cpp
    )
    target_link_libraries(${PROJECT_NAME}-fake-temperature-data-test ${PROJECT_NAME})
endif()

ament_package()
