#!/bin/bash

source /opt/ros/foxy/setup.bash
source "${HOME}"/march/ros2/install/local_setup.bash
cd "${HOME}"/march/ros2 || exit
echo "Launching ros2 with ROS_ARGS: ${ROS_ARGS} and ROS2_ARGS: ${ROS1_ARGS}"
exec ros2 launch march_launch march_simulation.launch.py ${ROS_ARGS} ${ROS2_ARGS}
