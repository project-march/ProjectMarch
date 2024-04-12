cmake_minimum_required(VERSION 3.5)
project(march_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "")

find_package(ament_cmake REQUIRED)
find_package(soem REQUIRED)
find_package(urdf REQUIRED)
find_package(march_safety REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(march_logger_cpp REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(soem march_logger_cpp march_safety)
ament_export_libraries(${PROJECT_NAME})

include(cmake/${PROJECT_NAME}-extras.cmake)

include_directories(
    include
    SYSTEM
    ${soem_INCLUDE_DIRS}/soem
)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating march_hardware RUNPATH to dependencies, in contrast to RPATH
set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

if(BUILD_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
    SHARED
    include/${PROJECT_NAME}/encoder/absolute_encoder.h
    include/${PROJECT_NAME}/encoder/encoder.h
    include/${PROJECT_NAME}/encoder/incremental_encoder.h
    include/${PROJECT_NAME}/error/error_type.h
    include/${PROJECT_NAME}/error/hardware_exception.h
    include/${PROJECT_NAME}/error/motor_controller_error.h
    include/${PROJECT_NAME}/ethercat/ethercat_master.h
    include/${PROJECT_NAME}/ethercat/pdo_interface.h
    include/${PROJECT_NAME}/ethercat/odrive_pdo_map.h
    include/${PROJECT_NAME}/ethercat/pdo_types.h
    include/${PROJECT_NAME}/ethercat/sdo_interface.h
    include/${PROJECT_NAME}/ethercat/slave.h
    include/${PROJECT_NAME}/motor_controller/actuation_mode.h
    include/${PROJECT_NAME}/motor_controller/motor_controller.h
    include/${PROJECT_NAME}/motor_controller/motor_controller_type.h
    include/${PROJECT_NAME}/motor_controller/motor_controller_state.h
    include/${PROJECT_NAME}/motor_controller/odrive/odrive.h
    include/${PROJECT_NAME}/motor_controller/odrive/odrive_state.h
    include/${PROJECT_NAME}/joint.h
    include/${PROJECT_NAME}/march_robot.h
    include/${PROJECT_NAME}/temperature/temperature_ges.h
    include/${PROJECT_NAME}/temperature/temperature_sensor.h
    include/${PROJECT_NAME}/torque_sensor/torque_sensor.h
    include/${PROJECT_NAME}/power_distribution_board/power_distribution_board.h
    src/encoder/absolute_encoder.cpp
    src/encoder/encoder.cpp
    src/encoder/incremental_encoder.cpp
    src/error/error_type.cpp
    src/error/motor_controller_error.cpp
    src/ethercat/ethercat_master.cpp
    src/ethercat/pdo_interface.cpp
    src/ethercat/odrive_pdo_map.cpp
    src/ethercat/sdo_interface.cpp
    src/motor_controller/odrive/odrive.cpp
    src/motor_controller/motor_controller.cpp
    src/joint.cpp
    src/march_robot.cpp
    src/temperature/temperature_ges.cpp
    src/torque_sensor/torque_sensor.cpp
    src/power_distribution_board/power_distribution_board.cpp
)

target_link_libraries(${PROJECT_NAME} pthread)
ament_target_dependencies(${PROJECT_NAME} soem march_logger_cpp march_safety)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)

# Add gtest based cpp test target and link libraries
 if(BUILD_TESTING)
     ament_add_gmock(${PROJECT_NAME}_test
         test/encoder/absolute_encoder_test.cpp
         test/encoder/encoder_test.cpp
         test/encoder/incremental_encoder_test.cpp
         test/error/hardware_exception_test.cpp
         test/ethercat/slave_test.cpp
#         test/joint_test.cpp
         test/mocks/mock_absolute_encoder.h
         test/mocks/mock_encoder.h
         test/mocks/mock_odrive.h
         test/mocks/mock_incremental_encoder.h
         test/mocks/mock_motorcontroller_state.h
         test/mocks/mock_pdo_interface.h
         test/mocks/mock_sdo_interface.h
         test/mocks/mock_slave.h
         test/mocks/mock_temperature_ges.h
         test/power_distribution_board/power_distribution_board_test.cpp
         test/temperature/temperature_ges_test.cpp
         test/torque_sensor/torque_sensor_test.cpp
         test/test_runner.cpp
     )
     target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
     ament_target_dependencies(${PROJECT_NAME}_test soem urdf march_safety)

     if(ENABLE_COVERAGE_TESTING)
         set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/check/*")
         add_code_coverage(
             NAME coverage_report
             DEPENDENCIES ${PROJECT_NAME}_test
         )
     endif()
 endif()
