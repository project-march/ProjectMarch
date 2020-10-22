cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY minimal airgait-v test_versions-v training-v
    DESTINATION share/${PROJECT_NAME}
)

ament_package()