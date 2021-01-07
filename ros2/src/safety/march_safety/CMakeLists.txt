cmake_minimum_required(VERSION 3.5)
project(march_safety)

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
find_package(march_shared_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ros_testing REQUIRED)

add_library(${PROJECT_NAME}
        src/input_device_safety.cpp
        src/safety_handler.cpp
        src/temperature_safety.cpp
)
ament_target_dependencies(${PROJECT_NAME} march_shared_msgs rclcpp sensor_msgs std_msgs urdf ros_testing)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# the executable that is run in ROS and links with the library defined above
add_executable(${PROJECT_NAME}_node src/safety_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node march_shared_msgs rclcpp sensor_msgs std_msgs urdf ros_testing)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

add_ros_test(test/launch/march_safety_connection_lost.launch.py)
# Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#
#    add_rostest_gtest(${PROJECT_NAME}_connection_lost_test
#        test/launch/${PROJECT_NAME}_connection_lost.test
#        test/rostest/connection_lost_test.cpp
#    )
#
#    add_rostest_gtest(${PROJECT_NAME}_connection_never_started_test
#        test/launch/${PROJECT_NAME}_connection_never_started.test
#        test/rostest/connection_never_started_test.cpp
#    )
#
#    add_rostest_gtest(${PROJECT_NAME}_connection_not_lost_test
#        test/launch/${PROJECT_NAME}_connection_not_lost.test
#        test/rostest/connection_not_lost_test.cpp
#    )
#    target_link_libraries(${PROJECT_NAME}_connection_lost_test ${catkin_LIBRARIES})
#    target_link_libraries(${PROJECT_NAME}_connection_never_started_test ${catkin_LIBRARIES})
#    target_link_libraries(${PROJECT_NAME}_connection_not_lost_test ${catkin_LIBRARIES})
#
#    if(ENABLE_COVERAGE_TESTING)
#        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
#        add_code_coverage(
#            NAME coverage_report
#            DEPENDENCIES
#            ${PROJECT_NAME}_connection_lost_test
#            ${PROJECT_NAME}_connection_never_started_test
#            ${PROJECT_NAME}_connection_not_lost_test
#        )
#    endif()
#endif()

ament_package()