cmake_minimum_required(VERSION 2.8.3)
project(march_ethercat)

find_package(catkin REQUIRED
        COMPONENTS
                march_shared_resources
                roscpp
                std_msgs
                std_srvs
        )

catkin_package(
        CATKIN_DEPENDS
                roscpp
                std_msgs
                std_srvs
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        soem
        )

set(SOURCES
        src/EthercatNode.cpp
        src/EthercatMaster.cpp
        src/EthercatSDO.cpp
        src/ethercat_io.cpp
        src/ethercat_safety.cpp
        src/Slaves/Slave.cpp
        src/Slaves/GES.cpp
        src/Slaves/IMC.cpp
        src/Slaves/PDB.cpp
        src/Slaves/TemplateGES.cpp
        )

file(GLOB SOEM_SRC
        soem/*.h
        soem/*.c
        )

add_executable(march_ethercat_node ${SOURCES} ${SOEM_SRC})

add_dependencies(march_ethercat_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(march_ethercat_node ${catkin_LIBRARIES})

# Give sudo access to ethernet port
add_custom_command( TARGET march_ethercat_node
                    POST_BUILD
                    COMMAND sudo setcap cap_net_raw+ep march_ethercat_node
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/march_ethercat
                  )