cmake_minimum_required(VERSION 2.8.3)
project(march_fake_sensor_data)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    march_shared_resources
    roscpp
    roslint
    sensor_msgs
    std_msgs
)

generate_dynamic_reconfigure_options(
    config/Temperatures.cfg
)

catkin_package(
    CATKIN_DEPENDS
    dynamic_reconfigure
    march_shared_resources
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
    "src/*.cpp"
    "src/*.h"
)

roslint_cpp(${lintfiles})

add_executable(march_fake_temperature_data_node src/FakeTemperatureData.cpp)

add_dependencies(march_fake_temperature_data_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_fake_temperature_data_node ${catkin_LIBRARIES})

install(TARGETS march_fake_temperature_data_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
