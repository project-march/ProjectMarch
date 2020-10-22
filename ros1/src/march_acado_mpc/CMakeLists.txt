cmake_minimum_required(VERSION 3.0.2)
project(march_acado_mpc)


find_package(catkin REQUIRED COMPONENTS
    march_shared_resources
    std_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
  march_shared_resources
  std_msgs
  roscpp
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)