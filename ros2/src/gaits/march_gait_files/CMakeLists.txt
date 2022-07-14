cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        airgait_vi
        test_joint_rotational_gaits
        test_joint_linear_gaits
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
