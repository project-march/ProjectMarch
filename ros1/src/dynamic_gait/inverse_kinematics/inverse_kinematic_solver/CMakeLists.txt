cmake_minimum_required(VERSION 3.0.2)
project(inverse_kinematic_solver)

find_package(catkin REQUIRED COMPONENTS rospy march_shared_msgs moveit_msgs)

catkin_package(
 CATKIN_DEPENDS rospy march_shared_msgs moveit_msgs
)

catkin_install_python(PROGRAMS src/inverse_kinematic_solver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})