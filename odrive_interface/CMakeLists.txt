cmake_minimum_required(VERSION 2.8.3)
project(odrive_interface)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
