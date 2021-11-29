#!/bin/bash

source /opt/ros/foxy/setup.bash
source "${HOME}"/march/ros2/install/local_setup.bash
cd "${HOME}"/march/ros2 || exit
exec ros2 launch march_launch march_simulation.launch.py
