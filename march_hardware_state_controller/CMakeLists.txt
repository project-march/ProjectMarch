cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_state_controller)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
    code_coverage
    controller_interface
    hardware_interface
    march_hardware_interface
    pluginlib
    realtime_tools
    roscpp
    roslint
    rostest
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    controller_interface
    hardware_interface
    march_hardware_interface
    pluginlib
    realtime_tools
    roscpp
)

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "*.cpp"
    "*.h"
)

roslint_cpp(${lintfiles})

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/march_hardware_state_controller.h
    src/march_hardware_state_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES march_hardware_state_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    add_rostest_gtest(${PROJECT_NAME}-test
        test/${PROJECT_NAME}.test
        test/TestRunner.cpp
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
