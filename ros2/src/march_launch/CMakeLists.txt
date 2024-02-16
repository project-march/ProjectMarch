cmake_minimum_required(VERSION 3.5)
project(march_launch)

find_package(ament_cmake REQUIRED)
find_package(plotjuggler REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
