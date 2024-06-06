cmake_minimum_required(VERSION 3.5)
project(march_fuzzy_generator)

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
find_package(march_shared_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.6)

add_library(${PROJECT_NAME} STATIC src/fuzzy_generator.cpp src/fuzzy_generator_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${YAMLCPP_LIBRARIES})

ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
        ament_index_cpp
)

add_executable(fuzzy_generator_node src/fuzzy_generator_node.cpp src/fuzzy_generator.cpp)

target_link_libraries(fuzzy_generator_node ${PROJECT_NAME} ${YAMLCPP_LIBRARIES})

ament_target_dependencies(
        fuzzy_generator_node
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
        ament_index_cpp
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS fuzzy_generator_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()