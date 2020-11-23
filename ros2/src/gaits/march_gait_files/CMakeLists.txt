cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

# TEMPORARY SOLUTION, WHEN WE TRANSFER COMPLETELY THIS WILL BE REPLACED
install(DIRECTORY "../../../../ros1/src/gaits/march_gait_files/"
    DESTINATION share/${PROJECT_NAME}
    PATTERN "package.xml" EXCLUDE
    PATTERN "CMakeLists.txt" EXCLUDE
)

ament_package()
