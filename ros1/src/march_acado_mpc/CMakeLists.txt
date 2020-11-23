cmake_minimum_required(VERSION 3.18)

project(march_acado_mpc DESCRIPTION "Model predictive control with the help of ACADO"
    LANGUAGES CXX C)

# Enable the FetchContent CMake module.
include(FetchContent)

# Define which packages this package depends on
find_package(catkin REQUIRED 
        COMPONENTS
        march_shared_resources
        roscpp
        std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    # Define the catkin projects that this project depends on. If another project
    # does "find_package(march_acado_mpc)" these packages will be seen as
    # transitive dependencies.
    CATKIN_DEPENDS march_shared_resources roscpp std_msgs
)

################
# ACADO SOLVER #
################
# Fetch the latest version of the acado solver from the Project MARCH GitLab. This
# is downloaded in the _deps/acado-solver-src folder in the build directory.
fetchcontent_declare(acado-solver
    GIT_REPOSITORY "https://gitlab.com/project-march/libraries/acado-solver.git"
    GIT_TAG "main"
    GIT_SHALLOW true
)
if(NOT acado-solver_POPULATED)
    fetchcontent_populate(acado-solver)
endif()

# Define a variable that points to the location of the ACADO. This
# location is chosen by FetchContent_Declare.
set(ACADO_DIR "${PROJECT_BINARY_DIR}/_deps/acado-solver-src")

# Create a list of all source files in the ACADO directory and store it
# in ACADO_SRCS.
aux_source_directory("${ACADO_DIR}" ACADO_SRCS)

###########
# QPOASES #
###########
# qpOASES has a version on GitHub, but it is not the version that is needed. We
# need qpOASES that is included with acado.
fetchcontent_declare(acado
    GIT_REPOSITORY "https://github.com/acado/acado.git"
    GIT_TAG "stable"
    GIT_SHALLOW true
)
if(NOT acado_POPULATED)
    fetchcontent_populate(acado)
endif()

# Define a variable that points to the location of the qpOASES. This
# location is chosen by FetchContent_Declare.
set(QPOASES_DIR "${PROJECT_BINARY_DIR}/_deps/acado-src/external_packages/qpoases")

# Create a list of all source files in the qpOASES source folder and store it
# in QPOASES_SRCS.
aux_source_directory("${QPOASES_DIR}/SRC" QPOASES_SRCS)

##################################################
# ACADO LIBRARY WITH BOTH THE SOLVER AND QPOASES #
##################################################
include_directories(
        include
        "${catkin_INCLUDE_DIRS}"
        "${ACADO_DIR}"

        # "lib/qpoases" is needed for acado_qpoases_interface to correctly build
        # as this generated file includes "INCLUDE/QProblem.hpp".
        ${QPOASES_DIR}
        ${QPOASES_DIR}/INCLUDE
        ${QPOASES_DIR}/SRC
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

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
