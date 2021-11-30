#!/bin/bash

SLEEP_TIME=20s
DONE_FILE=install/.done
cd "${HOME}"/march/ros2 || exit
rm "${DONE_FILE}"
source /opt/ros/foxy/setup.bash

while ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; do
  echo "Ros1 is not done yet, sleeping for ${SLEEP_TIME}. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  sleep $SLEEP_TIME;
done;

colcon build
touch "${DONE_FILE}"

