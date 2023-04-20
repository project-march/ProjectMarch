cmake_minimum_required(VERSION 3.5)
project(march_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config xacro
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
