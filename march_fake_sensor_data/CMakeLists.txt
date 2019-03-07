cmake_minimum_required(VERSION 2.8.3)
project(march_fake_sensor_data)
add_definitions(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        march_shared_resources
        roscpp
        roslint
        sensor_msgs
        std_msgs
        )

generate_dynamic_reconfigure_options(
        config/temperatures.cfg
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Roslinter
set(ROSLINT_CPP_OPTS "--filter=-build/include,-build/header_guard,-runtime/references" "--linelength=120")

file(GLOB_RECURSE lintfiles
        "src/*.cpp"
        "src/*.h"
        )

roslint_cpp(${lintfiles})

add_executable(march_fake_temperature_data_node
        src/FakeTemperatureData.cpp
        )

add_dependencies(march_fake_temperature_data_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_fake_temperature_data_node ${catkin_LIBRARIES})
