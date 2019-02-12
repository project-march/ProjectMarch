cmake_minimum_required(VERSION 2.8.3)
project(march_ethercat)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs roscpp std_msgs std_srvs)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(soem)

set(SOURCES EthercatNode.cpp EthercatMaster.cpp EthercatMaster.h ethercat_SDO.cpp ethercat_io.cpp ethercat_safety.cpp Slave.cpp Slave.h GES.cpp GES.h IMC.cpp IMC.h PDB.cpp PDB.h TemplateGES.cpp TemplateGES.h)

file(GLOB_RECURSE SOEM_SRC
    "*.h" "*.c"
)

add_library(${SOURCES} ${SOEM_SRC})

add_executable(march_ethercat_node ${SOURCES} ${SOEM_SRC})

add_dependencies(march_ethercat_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(march_ethercat_node ${catkin_LIBRARIES})

# Give sudo access to ethernet port
add_custom_command( TARGET march_ethercat_node
                    POST_BUILD
                    COMMAND sudo setcap cap_net_raw+ep march_ethercat_node
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../devel/lib/march_ethercat
                  )