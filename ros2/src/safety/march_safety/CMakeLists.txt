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

# add_library(${PROJECT_NAME} SHARED
# src/safety_node.cpp
# )

add_executable(${PROJECT_NAME}_node src/safety_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node 
  march_shared_msgs
  rclcpp
  std_msgs
  march_utility
)

include_directories(include)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
# install(TARGETS ${PROJECT_NAME}_node
#         RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

# if(BUILD_TESTING)
#         find_package(ament_cmake_gtest REQUIRED)
#         ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

#         set(TESTFILES
#           test/test_runner.cpp
#           test/safety_node_test.cpp
#         )

#         ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

#         target_link_directories(${PROJECT_NAME}_test my_lib)

#         target_include_directories(${PROJECT_NAME}_test PUBLIC
#           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#           $<INSTALL_INTERFACE:include>
#         )
#         ament_target_dependencies(${PROJECT_NAME}_test
#           std_msgs
#         )
#         target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
#       endif()

ament_package()
