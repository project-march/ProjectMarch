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
# in addition to not propagating march_hardware RUNPATH to dependencies, in contrast to RPATH
set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/encoder/absolute_encoder.h
    include/${PROJECT_NAME}/encoder/encoder.h
    include/${PROJECT_NAME}/encoder/incremental_encoder.h
    include/${PROJECT_NAME}/error/error_type.h
    include/${PROJECT_NAME}/error/hardware_exception.h
    include/${PROJECT_NAME}/error/motion_error.h
    include/${PROJECT_NAME}/ethercat/ethercat_master.h
    include/${PROJECT_NAME}/ethercat/pdo_interface.h
    include/${PROJECT_NAME}/ethercat/pdo_map.h
    include/${PROJECT_NAME}/ethercat/pdo_types.h
    include/${PROJECT_NAME}/ethercat/sdo_interface.h
    include/${PROJECT_NAME}/ethercat/slave.h
    include/${PROJECT_NAME}/imotioncube/actuation_mode.h
    include/${PROJECT_NAME}/imotioncube/imotioncube.h
    include/${PROJECT_NAME}/imotioncube/imotioncube_state.h
    include/${PROJECT_NAME}/imotioncube/imotioncube_target_state.h
    include/${PROJECT_NAME}/power/boot_shutdown_offsets.h
    include/${PROJECT_NAME}/power/high_voltage.h
    include/${PROJECT_NAME}/power/low_voltage.h
    include/${PROJECT_NAME}/power/net_driver_offsets.h
    include/${PROJECT_NAME}/power/net_monitor_offsets.h
    include/${PROJECT_NAME}/power/power_distribution_board.h
    include/${PROJECT_NAME}/temperature/temperature_ges.h
    include/${PROJECT_NAME}/temperature/temperature_sensor.h
    src/Joint.cpp
    src/MarchRobot.cpp
    src/encoder/absolute_encoder.cpp
    src/encoder/encoder.cpp
    src/encoder/incremental_encoder.cpp
    src/error/error_type.cpp
    src/error/motion_error.cpp
    src/ethercat/ethercat_master.cpp
    src/ethercat/pdo_interface.cpp
    src/ethercat/pdo_map.cpp
    src/ethercat/sdo_interface.cpp
    src/imotioncube/imotioncube.cpp
    src/imotioncube/imotioncube_target_state.cpp
    src/power/high_voltage.cpp
    src/power/low_voltage.cpp
    src/power/power_distribution_board.cpp
    src/temperature/temperature_ges.cpp
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
        test/mocks/MockAbsoluteEncoder.h
        test/mocks/MockEncoder.h
        test/mocks/MockIMotionCube.h
        test/mocks/MockIncrementalEncoder.h
        test/mocks/MockJoint.h
        test/mocks/MockPdoInterface.h
        test/mocks/MockSdoInterface.h
        test/mocks/MockSlave.h
        test/mocks/MockTemperatureGES.h
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
