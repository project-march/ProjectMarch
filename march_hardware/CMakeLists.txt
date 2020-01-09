cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    soem
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}/soem
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME}
    src/Encoder.cpp
    src/EtherCAT/EthercatIO.cpp
    src/EtherCAT/EthercatMaster.cpp
    src/EtherCAT/EthercatSDO.cpp
    src/HighVoltage.cpp
    src/IMotionCube.cpp
    src/IMotionCubeTargetState.cpp
    src/Joint.cpp
    src/LowVoltage.cpp
    src/MarchRobot.cpp
    src/PDOmap.cpp
    src/PowerDistributionBoard.cpp
    src/TemperatureGES.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(slave_count_check check/SlaveCount.cpp)
target_link_libraries(slave_count_check ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS slave_count_check
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(code_coverage REQUIRED)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    add_rostest_gmock(${PROJECT_NAME}-test
        test/march4cpp.test
        test/TestRunner.cpp
        test/TestEncoder.cpp
        test/TestIMotionCube.cpp
        test/TestTemperatureGES.cpp
        test/TestJoint.cpp
        test/TestPDOmap.cpp
        test/TestBootShutdownOffsets.cpp
        test/TestNetDriverOffsets.cpp
        test/TestNetMonitorOffsets.cpp
        test/TestLowVoltage.cpp
        test/TestHighVoltage.cpp
        test/TestPowerDistributionBoard.cpp
        test/mocks/MockTemperatureGES.cpp
        test/mocks/MockTemperatureSensor.cpp
        test/mocks/MockEncoder.cpp
        test/mocks/MockIMotionCube.cpp
        test/mocks/MockJoint.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} gtest)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
