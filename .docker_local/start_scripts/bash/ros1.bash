#!/bin/bash

source /opt/ros/noetic/setup.bash
if [ -f "${HOME}"/march/ros1/install/local_setup.bash ];then
  source "${HOME}"/march/ros1/install/local_setup.bash
fi
exec bash
