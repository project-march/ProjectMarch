cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_builder)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    march_hardware
    roscpp
    roslib
    roslint
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    LIBRARIES ${PROJECT_NAME}
)

set(ROSLINT_CPP_OPTS "--linelength=120")

file(GLOB_RECURSE lintfiles
    "*.cpp"
    "*.h"
)

roslint_cpp(${lintfiles})


include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/hardware_builder.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY robots
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    find_package(code_coverage REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()

    catkin_add_gtest(${PROJECT_NAME}_test
        test/test_runner.cpp
        test/allowed_robot_test.cpp
        test/encoder_builder_test.cpp
        test/imc_builder_test.cpp
        test/joint_builder_test.cpp
        test/pdb_builder_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME} gtest gmock)

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
        )
    endif()
endif ()
