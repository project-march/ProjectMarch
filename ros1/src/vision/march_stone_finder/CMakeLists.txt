cmake_minimum_required(VERSION 3.0.2)
project(march_stone_finder)

find_package(catkin REQUIRED rospy dynamic_reconfigure march_shared_msgs visualization_msgs)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    config/parameters.cfg
)

catkin_package(CATKIN_DEPENDS march_shared_msgs visualization_msgs)

# make sure configure headers are built before any node using them
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

catkin_install_python(PROGRAMS scripts/march_stone_finder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
