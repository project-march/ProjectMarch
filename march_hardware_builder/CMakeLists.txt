cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_builder)

add_definitions(-std=c++11 -Wall -g)


find_package(catkin REQUIRED COMPONENTS
        code_coverage
        roscpp
        roslint
        roslib
        rostest
        march_hardware
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        LIBRARIES
        march_hardware_builder
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "*.cpp"
        "*.h"
)

roslint_cpp(${lintfiles})


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

add_library(march_hardware_builder
        include/${PROJECT_NAME}/HardwareBuilder.h
        include/${PROJECT_NAME}/HardwareConfigExceptions.h
        include/${PROJECT_NAME}/AllowedRobot.h
        src/HardwareBuilder.cpp
)

target_link_libraries(march_hardware_builder
        ${catkin_LIBRARIES}
        yaml-cpp
)

install(TARGETS march_hardware_builder
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        APPEND_COVERAGE_COMPILER_FLAGS()
    endif()

    add_rostest_gtest(${PROJECT_NAME}-test
            test/march_hardware_builder.test
            test/TestRunner.cpp
            test/TestAllowedRobots.cpp
            test/TestHardwareBuilderCreateEncoder.cpp
            test/TestHardwareBuilderCreateIMotionCube.cpp
            test/TestHardwareBuilderCreateJoint.cpp
            test/TestHardwareBuilderCreatePowerDistributionBoard.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_hardware_builder gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
                NAME ${PROJECT_NAME}_coverage
                DEPENDS tests
        )
    endif()
endif ()
