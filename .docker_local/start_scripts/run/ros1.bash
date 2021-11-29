#!/bin/bash

source /opt/ros/noetic/setup.bash
source "${HOME}"/march/ros1/install/local_setup.bash
cd "${HOME}"/march/ros1/ || exit
exec roslaunch march_launch march_simulation.launch
