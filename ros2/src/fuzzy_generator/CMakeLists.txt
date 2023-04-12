cmake_minimum_required(VERSION 3.5)
project(fuzzy_generator)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
        STATIC
        include/${PROJECT_NAME}/fuzzy_generator.hpp
        include/${PROJECT_NAME}/fuzzy_node.hpp
        src/fuzzy_generator.cpp
        src/fuzzy_node.cpp
        )

ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
)

add_executable(fuzzy_node src/fuzzy_node.cpp src/fuzzy_generator.cpp include/${PROJECT_NAME}/fuzzy_generator.hpp
        include/${PROJECT_NAME}/fuzzy_node.hpp)
target_include_directories(fuzzy_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        fuzzy_node
        "rclcpp"
        "march_shared_msgs"
        "geometry_msgs"
)

install(TARGETS fuzzy_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

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
