cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    control_toolbox
    controller_manager
    joint_limits_interface
    march_description
    march_hardware
    march_hardware_builder
    march_shared_resources
    realtime_tools
    roscpp
    roslint
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

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "src/*.cpp"
    "src/*.h"
)

roslint_cpp(${lintfiles})


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    include/${PROJECT_NAME}/PowerNetOnOffCommand.h
    src/march_hardware_interface.cpp
    src/march_hardware_interface_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

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
        test/TestPdbStateInterface.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
