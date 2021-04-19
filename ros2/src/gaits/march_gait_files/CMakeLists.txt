cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-v minimal simulation_only-vi test_versions-vi training-v test_joint_gaits
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
