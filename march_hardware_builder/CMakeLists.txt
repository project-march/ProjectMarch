cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_builder)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    march_hardware
    roscpp
    roslib
    roslint
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    LIBRARIES ${PROJECT_NAME}
)

set(ROSLINT_CPP_OPTS "--linelength=120")

file(GLOB_RECURSE lintfiles
    "*.cpp"
    "*.h"
)

roslint_cpp(${lintfiles})


include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/HardwareBuilder.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY robots
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(code_coverage REQUIRED)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    add_rostest_gtest(${PROJECT_NAME}-test
        test/${PROJECT_NAME}.test
        test/TestRunner.cpp
        test/TestAllowedRobots.cpp
        test/TestHardwareBuilderCreateEncoder.cpp
        test/TestHardwareBuilderCreateIMotionCube.cpp
        test/TestHardwareBuilderCreateJoint.cpp
        test/TestHardwareBuilderCreatePowerDistributionBoard.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
