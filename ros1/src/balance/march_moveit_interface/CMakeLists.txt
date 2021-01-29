cmake_minimum_required(VERSION 3.0.2)
project(march_moveit_interface)

find_package(catkin REQUIRED COMPONENTS
  march_shared_msgs
  moveit_commander
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    march_shared_msgs
    moveit_commander
    rospy
    sensor_msgs
    geometry_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
