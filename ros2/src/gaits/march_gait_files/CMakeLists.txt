cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-v airgait-vi minimal no_translational-vi simulation_only-vi
        test_versions-vi training-v training-vi test_joint_gaits test_joint_linear_gaits
        test_left_odrive_upper test_left_odrive_lower
        test_rotational_simple teest_left_leg_rotational teest_right_leg_rotational exo_test
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
