cmake_minimum_required(VERSION 3.5)
project(march_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

install(DIRECTORY "../../../../ros1/src/shared/march_description/urdf"
        DESTINATION share/${PROJECT_NAME}
        )

set(XACRO_FILES
    "../../../../ros1/src/shared/march_description/urdf/march3.xacro"
    "../../../../ros1/src/shared/march_description/urdf/march4.xacro"
    "../../../../ros1/src/shared/march_description/urdf/test_joint_linear.xacro"
    "../../../../ros1/src/shared/march_description/urdf/test_joint_rotational.xacro"
)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

build_and_install_xacro_files(${XACRO_FILES})
