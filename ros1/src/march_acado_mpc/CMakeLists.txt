cmake_minimum_required(VERSION 3.0.2)
project(march_acado_mpc)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)
#SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
        control_msgs
        controller_interface
        controller_manager
        joint_state_controller
        joint_trajectory_controller
        march_hardware
        march_shared_resources
        std_msgs
        roscpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES $(PROJECT_NAME)
        CATKIN_DEPENDS
        control_msgs
        controller_interface
#        march_shared_resources
        std_msgs
        roscpp)

include_directories(include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${ACADO_INCLUDE_DIRS}
        lib/qpoases/INCLUDE/
        lib/qpoases/SRC/
        generated_src/
        include/march_acado_mpc/)

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

SET(march_acado_mpc_node_SRCS
        src/march_acado_mpc_node.cpp
        src/acado_mpc.cpp)

add_executable(march_acado_mpc_node ${march_acado_mpc_node_SRCS})
target_link_libraries(march_acado_mpc_node ${catkin_LIBRARIES})

#add_executable(acado_mpc
#        src/acado_mpc.cpp)
#target_link_libraries(acado_mpc ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
        src/march_acado_mpc_node.cpp
        src/acado_mpc.cpp)

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