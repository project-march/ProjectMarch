cmake_minimum_required(VERSION 3.5)
project(gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        march8_gaits
        test_joint_rotational_gaits
        test_joint_linear_gaits
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
