cmake_minimum_required(VERSION 3.5)
project(march_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
