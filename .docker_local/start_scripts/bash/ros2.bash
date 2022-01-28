#!/bin/bash

source /opt/ros/foxy/setup.bash
if [ -f "${HOME}"/march/ros2/install/local_setup.bash ];then
  source "${HOME}"/march/ros2/install/local_setup.bash
fi
exec bash
