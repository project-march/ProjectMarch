cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_robot_monitor)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_aggregator
  roscpp
  rospy
  std_msgs
)

catkin_package()
catkin_python_setup()

install(DIRECTORY launch resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
