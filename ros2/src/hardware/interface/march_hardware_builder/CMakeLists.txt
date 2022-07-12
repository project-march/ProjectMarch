cmake_minimum_required(VERSION 3.5)
project(march_hardware_builder)

#add_compile_options(-std=c++17 -Wall -Wextra -Werror)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(march_hardware REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(march_logger_cpp REQUIRED)

#if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
#    find_package(code_coverage REQUIRED)
#    # Add compiler flags for coverage instrumentation before defining any targets
#    append_coverage_compiler_flags()
#endif()
add_library(${PROJECT_NAME}
        src/hardware_builder.cpp
        )

ament_target_dependencies(
        ${PROJECT_NAME}
        march_hardware
        yaml-cpp
        march_logger_cpp
)
#target_link_libraries(${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        yaml-cpp
#        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


# These must be above the two installs otherwise can't be recognized by other packages.
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)  # For the ros_logger

# INSTALL

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
        )

install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()


### Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(${PROJECT_NAME}_test
#        test/absolute_encoder_builder_test.cpp
#        test/allowed_robot_test.cpp
#        test/imc_builder_test.cpp
#        test/motor_controller_builder_test.cpp
#        test/incremental_encoder_builder_test.cpp
#        test/joint_builder_test.cpp
#        test/test_runner.cpp
#    )
#    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#    if(ENABLE_COVERAGE_TESTING)
#        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
#        add_code_coverage(
#            NAME coverage_report
#            DEPENDENCIES ${PROJECT_NAME}_test
#        )
#    endif()
#endif()
