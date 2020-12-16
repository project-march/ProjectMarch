cmake_minimum_required(VERSION 3.0.2)
project(march_gait_files)

find_package(catkin REQUIRED)

catkin_package()

# TEMPORARY SOLUTION, WHEN WE TRANSFER COMPLETELY THIS WILL BE REPLACED
install(DIRECTORY "../../../../ros2/src/gaits/march_gait_files/"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "package.xml" EXCLUDE
    PATTERN "CMakeLists.txt" EXCLUDE
)