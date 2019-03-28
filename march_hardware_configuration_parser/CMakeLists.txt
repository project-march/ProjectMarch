cmake_minimum_required(VERSION 2.8.3)
project(march_hardware_configuration_parser)

add_definitions(-std=c++11 -Wall -g)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        rostest
        march_hardware
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        LIBRARIES
        march_hardware_configuration_parser
)

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)



add_library(march_hardware_configuration_parser
        include/${PROJECT_NAME}/HardwareBuilder.h
        include/${PROJECT_NAME}/HardwareConfigExceptions.h
        src/HardwareBuilder.cpp
        src/main.cpp
        )

target_link_libraries(march_hardware_configuration_parser ${catkin_LIBRARIES} yaml-cpp)


add_executable(hardware_configuration_parser_dummy src/main.cpp)
target_link_libraries(hardware_configuration_parser_dummy march_hardware_configuration_parser yaml-cpp)

install(TARGETS march_hardware_configuration_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Add gtest based cpp test target and link libraries
#if (CATKIN_ENABLE_TESTING)
#    add_rostest_gtest(${PROJECT_NAME}-test
#            )
#    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_hardware_configuration_parser gtest gmock)
#endif ()
