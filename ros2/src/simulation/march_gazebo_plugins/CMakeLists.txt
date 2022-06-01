cmake_minimum_required(VERSION 3.10.2)
project(march_gazebo_plugins)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/com_controller_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_package()