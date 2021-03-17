cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-v minimal simulation_only-vi test_versions-vi training-v lin_test_joint_gait rot_test_joint_gait
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
