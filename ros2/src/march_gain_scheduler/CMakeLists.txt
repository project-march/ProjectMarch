cmake_minimum_required(VERSION 3.5)
project(march_gain_scheduler)

# Default to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the source files for the library
set(GAIN_SCHEDULER_SOURCES
    src/gain_scheduler.cpp
    src/gain_scheduler_node.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_library(${PROJECT_NAME} STATIC ${GAIN_SCHEDULER_SOURCES})
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
else()
    add_library(${PROJECT_NAME} STATIC ${GAIN_SCHEDULER_SOURCES})
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# testing 
find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.6)


include_directories(include)# After finding yaml-cpp
include_directories(${YAMLCPP_INCLUDE_DIRS})

# Link yaml-cpp to your library
target_link_libraries(${PROJECT_NAME} ${YAMLCPP_LIBRARIES})

# target_include_directories should be placed after add_library
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "march_shared_msgs" "std_msgs" "geometry_msgs" "sensor_msgs" ament_index_cpp)

add_executable(gain_scheduler_node src/gain_scheduler_node.cpp src/gain_scheduler.cpp)

# Link yaml-cpp to your executable
target_link_libraries(gain_scheduler_node ${PROJECT_NAME} ${YAMLCPP_LIBRARIES})
# target_link_libraries(gainscheduler_node ${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS gain_scheduler_node EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
