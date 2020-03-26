cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    soem
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    LIBRARIES ${PROJECT_NAME}
    CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

include(cmake/${PROJECT_NAME}-extras.cmake)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${soem_INCLUDE_DIRS}/soem
)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating march_hardware_interface RUNPATH to dependencies, in contrast to RPATH
set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/encoder/AbsoluteEncoder.h
    include/${PROJECT_NAME}/encoder/Encoder.h
    include/${PROJECT_NAME}/encoder/IncrementalEncoder.h
    include/${PROJECT_NAME}/error/error_type.h
    include/${PROJECT_NAME}/error/hardware_exception.h
    include/${PROJECT_NAME}/error/motion_error.h
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
    src/encoder/AbsoluteEncoder.cpp
    src/encoder/Encoder.cpp
    src/encoder/IncrementalEncoder.cpp
    src/error/error_type.cpp
    src/error/motion_error.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pthread)

add_executable(slave_count_check check/slave_count.cpp)
target_link_libraries(slave_count_check ${PROJECT_NAME})
ros_enable_rpath(slave_count_check)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS slave_count_check
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gmock(${PROJECT_NAME}_test
        test/TestBootShutdownOffsets.cpp
        test/TestHighVoltage.cpp
        test/TestIMotionCube.cpp
        test/TestJoint.cpp
        test/TestLowVoltage.cpp
        test/TestNetDriverOffsets.cpp
        test/TestNetMonitorOffsets.cpp
        test/TestPDOmap.cpp
        test/TestPowerDistributionBoard.cpp
        test/TestRunner.cpp
        test/TestSlave.cpp
        test/TestTemperatureGES.cpp
        test/encoder/TestAbsoluteEncoder.cpp
        test/encoder/TestIncrementalEncoder.cpp
        test/encoder/TestEncoder.cpp
        test/error/test_hardware_exception.cpp
        test/error/test_motion_error.cpp
        test/mocks/MockAbsoluteEncoder.cpp
        test/mocks/MockEncoder.cpp
        test/mocks/MockIMotionCube.cpp
        test/mocks/MockIncrementalEncoder.cpp
        test/mocks/MockJoint.cpp
        test/mocks/MockTemperatureGES.cpp
        test/mocks/MockTemperatureSensor.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/check/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES ${PROJECT_NAME}_test
        )
    endif()
endif()
