cmake_minimum_required(VERSION 3.5)
project(march_hardware_builder)

# Set to C++17, because we make use of the 'optional' dependency.
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
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(include)

if(BUILD_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
        SHARED
        src/hardware_builder.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        ${PROJECT_NAME}
        march_hardware
        yaml-cpp
        march_logger_cpp
        rclcpp
)

# This is needed for some reason otherwise you can get linker errors.
# Normally this should have been done by ament_target_dependencies, .
target_link_libraries(${PROJECT_NAME}
        ament_index_cpp::ament_index_cpp
        ${YAML_CPP_LIBRARIES})

install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
        )

# EXPORT
# These must be above the two installs otherwise can't be recognized by other packages.
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        march_hardware
        yaml-cpp
        march_logger_cpp
        rclcpp
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


## Add gtest based cpp test target and link libraries
if(BUILD_TESTING)
    ament_add_gmock(${PROJECT_NAME}_test
       test/absolute_encoder_builder_test.cpp
        # test/test_runner.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES ${PROJECT_NAME}_test
        )
    endif()
endif()
ament_package()
