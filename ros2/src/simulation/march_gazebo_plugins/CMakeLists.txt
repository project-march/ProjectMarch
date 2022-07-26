cmake_minimum_required(VERSION 3.10.2)
project(march_gazebo_plugins)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/com_controller_plugin.cpp
  src/obstacle_controller.cpp
  src/com_controller_node.cpp
  )
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} yaml-cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME} march_shared_msgs rclcpp ament_index_cpp)
ament_package()