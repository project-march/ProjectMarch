cmake_minimum_required(VERSION 2.8.3)
project(march_imu_manager)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    roslint
    sensor_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    std_msgs
)

include_directories(
    include
    lib/include
    ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "src/*.cpp"
    "src/*.h"
)

roslint_cpp(${lintfiles})

set(xsensdeviceapi_LOCATION ${PROJECT_SOURCE_DIR}/lib/libxsensdeviceapi.so)
set(xstypes_LOCATION ${PROJECT_SOURCE_DIR}/lib/libxstypes.so)

find_library(xsensdeviceapi_LIBRARY NAMES xsensdeviceapi PATHS ${PROJECT_SOURCE_DIR}/lib)
find_library(xstypes_LIBRARY NAMES xstypes PATHS ${PROJECT_SOURCE_DIR}/lib)

add_library(xsensdeviceapi SHARED IMPORTED)
set_property(TARGET xsensdeviceapi PROPERTY IMPORTED_LOCATION ${xsensdeviceapi_LOCATION})

add_library(xstypes SHARED IMPORTED)
set_property(TARGET xstypes PROPERTY IMPORTED_LOCATION ${xstypes_LOCATION})

add_executable(${PROJECT_NAME}_node
    src/MtwCallback.cpp
    src/WirelessMaster.cpp
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${xsensdeviceapi_LIBRARY}
    ${xstypes_LIBRARY}
    ${catkin_LIBRARIES}
)

install(
    FILES
    ${xsensdeviceapi_LOCATION}
    ${xsensdeviceapi_LOCATION}.4
    ${xsensdeviceapi_LOCATION}.4.6.0
    ${xstypes_LOCATION}
    ${xstypes_LOCATION}.4
    ${xstypes_LOCATION}.4.6.0
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
