cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_interface)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    control_toolbox
    controller_manager
    joint_limits_interface
    joint_state_controller
    joint_trajectory_controller
    march_description
    march_hardware
    march_hardware_builder
    march_shared_resources
    realtime_tools
    roscpp
    std_msgs
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    controller_manager
    joint_limits_interface
    march_description
    march_hardware
    march_hardware_builder
    march_shared_resources
    realtime_tools
    roscpp
    std_msgs
    urdf
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

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/march_hardware_interface.cpp
    src/march_hardware_interface_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# From march_hardware/cmake
ros_enable_rpath(${PROJECT_NAME}_node)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test
        test/TestRunner.cpp
        test/TestPdbStateInterface.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} gtest)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES ${PROJECT_NAME}_test
        )
    endif()
endif()
