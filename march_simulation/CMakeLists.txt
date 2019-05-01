cmake_minimum_required(VERSION 2.8.3)
project(march_simulation)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  march_description
  robot_state_publisher
  xacro
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
