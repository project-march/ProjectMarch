cmake_minimum_required(VERSION 2.8.3)
project(march_imu_manager)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    sensor_msgs
)

include_directories(
    include
    SYSTEM
    lib/include
    ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

set(xsensdeviceapi_LOCATION ${PROJECT_SOURCE_DIR}/lib/libxsensdeviceapi.so)
set(xstypes_LOCATION ${PROJECT_SOURCE_DIR}/lib/libxstypes.so)

find_library(xsensdeviceapi_LIBRARY NAMES xsensdeviceapi PATHS ${PROJECT_SOURCE_DIR}/lib)
find_library(xstypes_LIBRARY NAMES xstypes PATHS ${PROJECT_SOURCE_DIR}/lib)

add_library(xsensdeviceapi SHARED IMPORTED)
set_property(TARGET xsensdeviceapi PROPERTY IMPORTED_LOCATION ${xsensdeviceapi_LOCATION})

add_library(xstypes SHARED IMPORTED)
set_property(TARGET xstypes PROPERTY IMPORTED_LOCATION ${xstypes_LOCATION})

add_library(${PROJECT_NAME}
    src/mtw.cpp
    src/wireless_master.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${xsensdeviceapi_LIBRARY}
    ${xstypes_LIBRARY}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
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

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(wireless_master_test test/wireless_master_test.cpp)
    target_link_libraries(wireless_master_test
        ${PROJECT_NAME}
        ${xsensdeviceapi_LIBRARY}
        ${xstypes_LIBRARY}
        ${catkin_LIBRARIES}
    )

    if(ENABLE_COVERAGE_TESTING)
        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/lib/*")
        add_code_coverage(
            NAME coverage_report
            DEPENDENCIES wireless_master_test
        )
    endif()
endif()
