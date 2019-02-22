cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rostest
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        LIBRARIES
        march4cpp
)

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

add_library(march4cpp
        ${SOEM_SRC}
        include/${PROJECT_NAME}/EtherCAT/EthercatIO.h
        include/${PROJECT_NAME}/EtherCAT/EthercatMaster.h
        include/${PROJECT_NAME}/EtherCAT/EthercatSDO.h
        include/${PROJECT_NAME}/AngleConversions.h
        include/${PROJECT_NAME}/Encoder.h
        include/${PROJECT_NAME}/IMotionCube.h
        include/${PROJECT_NAME}/Joint.h
        include/${PROJECT_NAME}/March4.h
        include/${PROJECT_NAME}/Slave.h
        include/${PROJECT_NAME}/TemperatureSensor.h
        src/EtherCAT/EthercatIO.cpp
        src/EtherCAT/EthercatMaster.cpp
        src/EtherCAT/EthercatSDO.cpp
        src/Encoder.cpp
        src/IMotionCube.cpp
        src/Joint.cpp
        src/March4.cpp
        src/TemperatureSensor.cpp
        src/main.cpp)

target_link_libraries(march4cpp ${catkin_LIBRARIES})


add_executable(hardware_dummy src/main.cpp)
target_link_libraries(hardware_dummy march4cpp)

# Give sudo access to ethernet port
add_custom_command(
        TARGET hardware_dummy
        POST_BUILD
        COMMAND sudo setcap cap_net_raw+ep hardware_dummy
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/march_hardware
)

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(${PROJECT_NAME}-test
            test/march4cpp.test
            test/TestRunner.cpp
            test/TestIMotionCube.cpp
            test/TestTemperatureSensor.cpp
            test/TestJoint.cpp
            test/mocks/MockTemperatureSensor.cpp
            )
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} march4cpp)
endif ()
