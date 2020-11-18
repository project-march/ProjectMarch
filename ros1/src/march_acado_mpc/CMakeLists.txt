cmake_minimum_required(VERSION 3.10)
PROJECT(march_acado_mpc)


find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        roscpp
        std_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        std_msgs
        roscpp
)

include_directories(
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        generated_src/

        # "lib/qpoases" is needed for acado_qpoases_interface to correctly build
        # as this generated file includes "INCLUDE/QProblem.hpp". This path would
        # only be correct if the qpoases directory was also located in generated_src
        lib/qpoases
        lib/qpoases/INCLUDE
        lib/qpoases/SRC


)

# Create an object that contains all ACADO sources
#set(ACADO_SRCS
#        generated_src/acado_auxiliary_functions.c
#        generated_src/acado_integrator.c
#        generated_src/acado_qpoases_interface.cpp
#        generated_src/acado_solver.c
#        )

# A recursive way of including all files above
file(GLOB ACADO_SRCS
        generated_src/*.c
        generated_src/*.cpp
        )

# Create an object that contains all qpOASES sources
#set(QPOASES_SRCS
#        lib/qpoases/SRC/Bounds.cpp
#        lib/qpoases/SRC/Constraints.cpp
#        lib/qpoases/SRC/CyclingManager.cpp
#        lib/qpoases/SRC/Indexlist.cpp
#        lib/qpoases/SRC/MessageHandling.cpp
#        lib/qpoases/SRC/QProblem.cpp
#        lib/qpoases/SRC/QProblemB.cpp
#        lib/qpoases/SRC/SubjectTo.cpp
#        lib/qpoases/SRC/Utils.cpp
#
#        lib/qpoases/SRC/Bounds.ipp
#        lib/qpoases/SRC/Constraints.ipp
#        lib/qpoases/SRC/CyclingManager.ipp
#        lib/qpoases/SRC/Indexlist.ipp
#        lib/qpoases/SRC/MessageHandling.ipp
#        lib/qpoases/SRC/QProblem.ipp
#        lib/qpoases/SRC/QProblemB.ipp
#        lib/qpoases/SRC/SubjectTo.ipp
#        lib/qpoases/SRC/Utils.ipp
#        )

# A recursive way of including all files above
file(GLOB_RECURSE QPOASES_SRCS
        lib/qpoases/SRC/*.cpp
        )

add_library(${PROJECT_NAME}
        src/acado_mpc.cpp
        ${ACADO_SRCS}
        ${QPOASES_SRCS}
        )


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
