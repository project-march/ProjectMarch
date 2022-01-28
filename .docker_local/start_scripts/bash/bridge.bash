#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

if [ -f "${HOME}"/march/ros1/install/local_setup.bash ];then
  source "${HOME}"/march/ros1/install/local_setup.bash
fi

if [ -f "${HOME}"/march/ros2/install/local_setup.bash ];then
  source "${HOME}"/march/ros2/install/local_setup.bash
fi

if [ -f "${HOME}"/ros1_bridge/install/local_setup.bash ];then
  source "${HOME}"/ros1_bridge/install/local_setup.bash
fi

exec bash