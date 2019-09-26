cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        code_coverage
        roscpp
        roslint
        rostest
        soem
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        soem
        LIBRARIES
        ${PROJECT_NAME}
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "include/*.h"
        )

roslint_cpp(${lintfiles})


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${soem_INCLUDE_DIRS}/soem
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

add_library(${PROJECT_NAME}
        include/${PROJECT_NAME}/ActuationMode.h
        include/${PROJECT_NAME}/PowerDistributionBoard.h
        include/${PROJECT_NAME}/IMotionCubeTargetState.h
        include/${PROJECT_NAME}/EtherCAT/EthercatIO.h
        include/${PROJECT_NAME}/EtherCAT/EthercatMaster.h
        include/${PROJECT_NAME}/EtherCAT/EthercatSDO.h
        include/${PROJECT_NAME}/AngleConversions.h
        include/${PROJECT_NAME}/Encoder.h
        include/${PROJECT_NAME}/IMotionCube.h
        include/${PROJECT_NAME}/IMotionCubeState.h
        include/${PROJECT_NAME}/Joint.h
        include/${PROJECT_NAME}/MarchRobot.h
        include/${PROJECT_NAME}/NetDriverOffsets.h
        include/${PROJECT_NAME}/Slave.h
        include/${PROJECT_NAME}/TemperatureGES.h
        include/${PROJECT_NAME}/TemperatureSensor.h
        include/${PROJECT_NAME}/LowVoltage.h
        include/${PROJECT_NAME}/HighVoltage.h
        include/${PROJECT_NAME}/BootShutdownOffsets.h
        include/${PROJECT_NAME}/PDOmap.h
        src/PowerDistributionBoard.cpp
        src/EtherCAT/EthercatIO.cpp
        src/EtherCAT/EthercatMaster.cpp
        src/EtherCAT/EthercatSDO.cpp
        src/LowVoltage.cpp
        src/HighVoltage.cpp
        src/Encoder.cpp
        src/IMotionCube.cpp
        src/Joint.cpp
        src/IMotionCubeTargetState.cpp
        src/MarchRobot.cpp
        src/TemperatureGES.cpp
        src/PDOmap.cpp
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(slave_count_check check/SlaveCount.cpp)
target_link_libraries(slave_count_check ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Give sudo access to ethernet port
add_custom_command(
        TARGET slave_count_check
        COMMAND sudo setcap cap_net_raw+ep slave_count_check
        WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
        POST_BUILD
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

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
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/soem*")
        add_code_coverage(
                NAME ${PROJECT_NAME}_coverage
                DEPENDS tests
        )
    endif()
endif ()
