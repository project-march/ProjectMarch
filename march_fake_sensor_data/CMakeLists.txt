cmake_minimum_required(VERSION 3.10.2)
project(march_fake_sensor_data LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    roscpp
    sensor_msgs
)

generate_dynamic_reconfigure_options(
    config/Temperatures.cfg
)

catkin_package(
    CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    sensor_msgs
)

add_executable(march_fake_temperature_data_node src/FakeTemperatureData.cpp)
target_include_directories(march_fake_temperature_data_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(march_fake_temperature_data_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(march_fake_temperature_data_node PRIVATE ${catkin_LIBRARIES})
target_compile_features(march_fake_temperature_data_node PRIVATE cxx_std_14)
target_compile_options(march_fake_temperature_data_node PRIVATE -Wall -Wextra -Werror)

install(TARGETS march_fake_temperature_data_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
