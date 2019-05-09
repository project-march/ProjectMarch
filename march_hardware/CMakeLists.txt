cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        rostest
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        LIBRARIES
        march_hardware
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
        soem
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)


file(GLOB SOEM_SRC
        soem/*.h
        soem/*.c
        )

add_library(march_hardware
        ${SOEM_SRC}
        include/${PROJECT_NAME}/PowerDistributionBoard.h
        include/${PROJECT_NAME}/EtherCAT/EthercatIO.h
        include/${PROJECT_NAME}/EtherCAT/EthercatMaster.h
        include/${PROJECT_NAME}/EtherCAT/EthercatSDO.h
        include/${PROJECT_NAME}/AngleConversions.h
        include/${PROJECT_NAME}/Encoder.h
        include/${PROJECT_NAME}/IMotionCube.h
        include/${PROJECT_NAME}/Joint.h
        include/${PROJECT_NAME}/MarchRobot.h
        include/${PROJECT_NAME}/NetDriverOffsets.h
        include/${PROJECT_NAME}/Slave.h
        include/${PROJECT_NAME}/TemperatureGES.h
        include/${PROJECT_NAME}/TemperatureSensor.h
        include/${PROJECT_NAME}/LowVoltageOffsets.h
        include/${PROJECT_NAME}/BootShutdownOffsets.h
        include/${PROJECT_NAME}/PDOmap.h
        src/PowerDistributionBoard.cpp
        src/EtherCAT/EthercatIO.cpp
        src/EtherCAT/EthercatMaster.cpp
        src/EtherCAT/EthercatSDO.cpp
        src/Encoder.cpp
        src/IMotionCube.cpp
        src/Joint.cpp
        src/MarchRobot.cpp
        src/TemperatureGES.cpp
        src/PDOmap.cpp
        src/main.cpp
        )

target_link_libraries(march_hardware ${catkin_LIBRARIES})


add_executable(hardware_dummy src/main.cpp)
target_link_libraries(hardware_dummy march_hardware)

install(TARGETS march_hardware
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Give sudo access to ethernet port
add_custom_command(
        TARGET hardware_dummy
        POST_BUILD
        COMMAND sudo setcap cap_net_raw+ep hardware_dummy
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/march_hardware
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gmock(${PROJECT_NAME}-test
            test/march4cpp.test
            test/TestRunner.cpp
            test/TestEncoder.cpp
            test/TestIMotionCube.cpp
            test/TestTemperatureGES.cpp
            test/TestJoint.cpp
            test/TestPDOmap.cpp
            test/mocks/MockTemperatureGES.cpp
            test/mocks/MockTemperatureSensor.cpp
            test/mocks/MockEncoder.cpp
            test/mocks/MockIMotionCube.cpp
            test/mocks/MockJoint.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march_hardware gtest)
endif ()
