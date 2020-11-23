cmake_minimum_required(VERSION 3.18)

project(march_acado_mpc DESCRIPTION "Project to perform model predictive control with the help of ACADO"
                        LANGUAGES CXX)

# Define which packages this package depends on
find_package(catkin REQUIRED 
        COMPONENTS
        march_shared_resources
        roscpp
        std_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        # Define the catkin projects that this project depends on. If another project does
        # "find_package(march_acado_mpc)" these packages will be seen as transitive dependencies.
        CATKIN_DEPENDS march_shared_resources roscpp std_msgs
)

################
# ACADO SOLVER #
################
# Fetch the latest version of the acado solver from the Project MARCH GitLab. This
# is downloaded in the _deps/acado-solver-src folder in the build directory.
include(FetchContent)
FetchContent_Declare(acado-solver
    GIT_REPOSITORY "https://gitlab.com/project-march/libraries/acado-solver.git"
    GIT_TAG "main"
)
FetchContent_MakeAvailable(acado-solver)

# Define a variable that points to the location of the ACADO. This
# location is chosen by FetchContent_Declare.
set(ACADO_DIR "${PROJECT_BINARY_DIR}/_deps/acado-solver-src")

file(GLOB_RECURSE ACADO_SRCS 
    RELATIVE "${ACADO_DIR}"
    CONFIGURE_DEPENDS
    *.c   
    *.cpp
)

##################################################
# ACADO LIBRARY WITH BOTH THE SOLVER AND QPOASES #
##################################################
include_directories(
        "include/${PROJECT_NAME}"
        "${catkin_INCLUDE_DIRS}"
        "${ACADO_DIR}"

        # "lib/qpoases" is needed for acado_qpoases_interface to correctly build
        # as this generated file includes "INCLUDE/QProblem.hpp". This path would
        # only be correct if the qpoases directory was also located in generated_src
        lib/qpoases
        lib/qpoases/INCLUDE
        lib/qpoases/SRC
)

# A recursive way of including all files above
file(GLOB_RECURSE QPOASES_SRCS
        lib/qpoases/SRC/*.cpp
        )

add_library(${PROJECT_NAME}
        src/acado_mpc.cpp
        "${QPOASES_SRCS}"
        "${ACADO_SRCS}"
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# ROS NODE #
############
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
