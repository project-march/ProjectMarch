cmake_minimum_required(VERSION 2.8.3)
project(ethercat)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs custom_msgs roscpp std_msgs std_srvs)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(soem)

set(SOURCES led_test.cpp update.cpp ethercat_io.cpp ros_init.cpp imc_callback.cpp ges_callback.cpp ethercat_safety.cpp pdb_callback.cpp ipd_callback.cpp launch_parameters.cpp)

file(GLOB_RECURSE SOEM_SRC
    "*.h" "*.c"
)

add_library(${SOURCES} ${SOEM_SRC})

add_executable(led_test ${SOURCES} ${SOEM_SRC})

add_dependencies(led_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(led_test ${catkin_LIBRARIES})
