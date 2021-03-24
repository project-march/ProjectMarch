cmake_minimum_required(VERSION 3.10.2)
project(march_gazebo_plugins LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    march_shared_msgs
    roscpp
)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} contact
    CATKIN_DEPENDS march_shared_msgs roscpp
)

add_library(${PROJECT_NAME}
    src/com_controller_plugin.cpp
    src/obstacle_controller.cpp
    src/walk_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS}/${PROJECT_NAME} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} yaml-cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#####
# Add contact plugin
#####
add_library(contact
    src/contact_plugin.cpp
)

target_include_directories(contact PRIVATE ${catkin_INCLUDE_DIRS}/contact ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(contact PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_compile_features(contact PRIVATE cxx_std_17)
target_compile_options(contact PRIVATE -Wall -Wextra -Werror)

install(DIRECTORY include/contact/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/contact
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS contact
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
