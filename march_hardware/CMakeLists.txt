cmake_minimum_required(VERSION 2.8.3)
project(march_hardware)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
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
        include/${PROJECT_NAME}/IMotionCube.h
        include/${PROJECT_NAME}/Joint.h
        include/${PROJECT_NAME}/March4.h
        include/${PROJECT_NAME}/TemperatureSensor.h
        src/IMotionCube.cpp
        src/Joint.cpp
        src/March4.cpp
        src/TemperatureSensor.cpp
        src/EtherCAT/EthercatMaster.cpp
        include/${PROJECT_NAME}/EtherCAT/EthercatSDO.h
        include/${PROJECT_NAME}/EtherCAT/EthercatMaster.h
        include/${PROJECT_NAME}/EtherCAT/EthercatIO.h
        src/EtherCAT/EthercatSDO.cpp
        src/EtherCAT/EthercatIO.cpp

        ${SOEM_SRC}


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