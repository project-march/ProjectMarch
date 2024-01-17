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
        add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(march_utility REQUIRED)

add_executable(march_safety_node src/safety_node.cpp)

ament_target_dependencies(march_safety_node
  march_shared_msgs
  rclcpp
  std_msgs
  march_utility
)

include_directories(include)

# Include include directory of package
target_include_directories(march_safety_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS march_safety_node
        DESTINATION lib/march_safety)

#add add package launch directory to ros2/install so it is visible when launching from ros2
install(DIRECTORY
        launch
        DESTINATION share/march_safety/
        )

# install(DIRECTORY include/
#   DESTINATION include
# )

# install(FILES package.xml
#   DESTINATION share/march_safety
# )


ament_package()
