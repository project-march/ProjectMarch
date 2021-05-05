cmake_minimum_required(VERSION 3.16)

project(march_acado_mpc DESCRIPTION "Model predictive control with the help of ACADO"
    LANGUAGES CXX C)

# Enable the FetchContent CMake module.
include(FetchContent)

# Define which packages this package depends on
find_package(catkin REQUIRED 
        COMPONENTS
        control_msgs
        controller_interface
        controller_manager
        joint_state_controller
        joint_trajectory_controller
        march_shared_msgs
        pluginlib
        roscpp
        std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    # Define the catkin projects that this project depends on. If another project
    # does "find_package(march_acado_mpc)" these packages will be seen as
    # transitive dependencies.
    CATKIN_DEPENDS
    control_msgs
    controller_interface
    march_shared_msgs
    pluginlib
    roscpp
    std_msgs
)

################
# ACADO SOLVER #
################
# Fetch the latest version of the acado solver from the Project MARCH GitLab. This
# is downloaded in the _deps/acado-solver-src folder in the build directory.
fetchcontent_declare(acado-solver
    GIT_REPOSITORY "https://gitlab.com/project-march/libraries/acado-solver.git"
    GIT_TAG "14-add-a-disconnected-full-exo-model"
    GIT_SHALLOW true
)
if(NOT acado-solver_POPULATED)
    fetchcontent_populate(acado-solver)
endif()

# Choose which model to use
# Models:   full_exo_simulation_disconnected
#           test_joint_rotational,
#           test_joint_linear
set(MODEL_NAME test_joint_rotational)

# Define a variable that points to the location of the chosen ACADO model_export directory. This
# location is chosen by FetchContent_Declare.
set(ACADO_DIR "${PROJECT_BINARY_DIR}/_deps/acado-solver-src/models/${MODEL_NAME}/model_export")


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
        "${QPOASES_SRCS}"
        "${ACADO_SRCS}"
        src/joint_trajectory_mpc.cpp
        src/model_predictive_controller.cpp
        src/mpc_references.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
