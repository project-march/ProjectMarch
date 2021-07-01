cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        march_v_gaits/airgait_v
        march_v_gaits/training_v
        march_v_gaits/test_versions_v
        airgait_vi
        minimal
        simulation_only_vi
        test_versions_vi
        training_vi
        test_joint_rotational_gaits
        test_joint_linear_gaits
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
