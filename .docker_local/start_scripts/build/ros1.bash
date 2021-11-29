#!/bin/bash

DONE_FILE=install/.done
cd "${HOME}"/march/ros1/ || exit
rm "${DONE_FILE}"
source /opt/ros/noetic/setup.bash
colcon build
exec touch "${DONE_FILE}"
