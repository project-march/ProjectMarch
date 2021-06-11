cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-vi minimal simulation_only-vi
        test_joint_gaits test_versions-vi training-vi
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
