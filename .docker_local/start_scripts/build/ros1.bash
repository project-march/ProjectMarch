#!/bin/bash

DONE_FILE=install/.done
cd "${HOME}"/march/ros1/ || exit
if [ -f "${DONE_FILE}" ]; then rm "${DONE_FILE}"; fi;
source /opt/ros/noetic/setup.bash
colcon build
exec touch ${DONE_FILE}
