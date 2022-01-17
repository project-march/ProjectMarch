#!/bin/bash

source /opt/ros/noetic/setup.bash
source "${HOME}"/march/ros1/install/local_setup.bash
cd "${HOME}"/march/ros1/ || exit
echo "Launching ros1 with ROS_ARGS: ${ROS_ARGS} and ROS1_ARGS: ${ROS1_ARGS}"
exec roslaunch march_launch march_simulation.launch ${ROS_ARGS} ${ROS1_ARGS}
