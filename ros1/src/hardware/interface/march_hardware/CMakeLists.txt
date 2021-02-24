cmake_minimum_required(VERSION 3.0.2)
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
    roscpp
    soem
    urdf
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
    include/${PROJECT_NAME}/joint.h
    include/${PROJECT_NAME}/march_robot.h
    include/${PROJECT_NAME}/power/boot_shutdown_offsets.h
    include/${PROJECT_NAME}/power/high_voltage.h
    include/${PROJECT_NAME}/power/low_voltage.h
    include/${PROJECT_NAME}/power/net_driver_offsets.h
    include/${PROJECT_NAME}/power/net_monitor_offsets.h
    include/${PROJECT_NAME}/power/power_distribution_board.h
    include/${PROJECT_NAME}/temperature/temperature_ges.h
    include/${PROJECT_NAME}/temperature/temperature_sensor.h
    include/${PROJECT_NAME}/pressure_sole/pressure_sole.h
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
    src/joint.cpp
    src/march_robot.cpp
    src/power/high_voltage.cpp
    src/power/low_voltage.cpp
    src/power/power_distribution_board.cpp
    src/temperature/temperature_ges.cpp
    src/pressure_sole/pressure_sole.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pthread)

add_executable(slave_count_check check/slave_count.cpp)
target_link_libraries(slave_count_check ${PROJECT_NAME})
ros_enable_rpath(slave_count_check)

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

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gmock(${PROJECT_NAME}_test
        test/encoder/absolute_encoder_test.cpp
        test/encoder/encoder_test.cpp
        test/encoder/incremental_encoder_test.cpp
        test/error/hardware_exception_test.cpp
        test/error/motion_error_test.cpp
        test/ethercat/pdo_map_test.cpp
        test/ethercat/slave_test.cpp
        test/imotioncube/imotioncube_test.cpp
        test/joint_test.cpp
        test/mocks/mock_absolute_encoder.h
        test/mocks/mock_encoder.h
        test/mocks/mock_imotioncube.h
        test/mocks/mock_incremental_encoder.h
        test/mocks/mock_joint.h
        test/mocks/mock_pdo_interface.h
        test/mocks/mock_sdo_interface.h
        test/mocks/mock_slave.h
        test/mocks/mock_temperature_ges.h
        test/power/boot_shutdown_offsets_test.cpp
        test/power/high_voltage_test.cpp
        test/power/low_voltage_test.cpp
        test/power/net_driver_offsets_test.cpp
        test/power/net_monitor_offsets_test.cpp
        test/power/power_distribution_board_test.cpp
        test/temperature/temperature_ges_test.cpp
        test/pressure_sole/pressure_sole_test.cpp
        test/test_runner.cpp
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
