cmake_minimum_required(VERSION 3.5)
project(march_footstep_planner)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


include_directories(include)

add_library(${PROJECT_NAME} STATIC
  src/footstep_planner.cpp 
  src/footstep_planner_node.cpp 
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  march_shared_msgs
  ament_index_cpp
)

add_executable(footstep_planner_node
  src/footstep_planner_node.cpp
  src/footstep_planner.cpp
)

ament_target_dependencies(footstep_planner_node
  rclcpp
  march_shared_msgs
  ament_index_cpp
  )

install(TARGETS footstep_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  launch test
  DESTINATION share/${PROJECT_NAME}/ 
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(march_shared_msgs REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/footstep_planner_test.cpp 
    test/test_runner.cpp 
    src/footstep_planner.cpp
    )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    ament_index_cpp
    march_shared_msgs
    trajectory_msgs
  )
endif()

# Add the necessary Python modules
ament_package()
