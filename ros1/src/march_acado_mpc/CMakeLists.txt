cmake_minimum_required(VERSION 3.0.2)
project(march_acado_mpc)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)
#SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        std_msgs
        roscpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES $(PROJECT_NAME)
        CATKIN_DEPENDS
        march_shared_resources
        std_msgs
        roscpp)


add_executable(march_acado_mpc_node
        src/march_acado_mpc_node.cpp)

include_directories(include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${ACADO_INCLUDE_DIRS}
        lib/qpoases/INCLUDE/
        lib/qpoases/SRC/
        generated_src/)

#SET (qpOASES_SRCS
#        lib/qpoases/SRC/Bounds.cpp
#
#        lib/qpoases/SRC/Constraints.cpp
#        lib/qpoases/SRC/CyclingManager.cpp
#        lib/qpoases/SRC/Indexlist.cpp
#        lib/qpoases/SRC/MessageHandling.cpp
#        lib/qpoases/SRC/QProblem.cpp
#        lib/qpoases/SRC/QProblemB.cpp
#        lib/qpoases/SRC/SubjectTo.cpp
#        lib/qpoases/SRC/Utils.cpp
#
#        include/${PROJECT_NAME}/acado_mpc.h)

add_library(${PROJECT_NAME}
        src/march_acado_mpc_node.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS march_acado_mpc_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES src/march_acado_mpc_node.cpp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})