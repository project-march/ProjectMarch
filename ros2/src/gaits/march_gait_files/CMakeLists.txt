cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-v airgait-vi minimal simulation_only-vi
        test_versions-vi training-v training-vi test_joint_rotational_gaits test_joint_linear_gaits no_translational-vi
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
