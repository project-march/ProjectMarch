cmake_minimum_required(VERSION 3.5)
project(march_plotjuggler_launcher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
