cmake_minimum_required(VERSION 2.8.3)
project(march_temperature_sensor_controller)

add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
  code_coverage
  controller_interface
  hardware_interface
  march_hardware_interface
  pluginlib
  realtime_tools
  roscpp
  roslint
  rostest
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    controller_interface
    hardware_interface
    march_hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "*.cpp"
        "*.h"
        )

roslint_cpp(${lintfiles})

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/march_temperature_sensor_controller.cpp
  include/march_temperature_sensor_controller/march_temperature_sensor_controller.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES march_temperature_sensor_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  add_rostest_gtest(${PROJECT_NAME}-test
          test/march_temperature_sensor_controller_test.test
          test/TestRunner.cpp
          )
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} gtest gmock)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
            NAME ${PROJECT_NAME}_coverage
            DEPENDS tests
    )
  endif()
endif ()