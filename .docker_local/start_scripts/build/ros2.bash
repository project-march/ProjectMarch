#!/bin/bash

SLEEP_TIME=20s
DONE_FILE=install/.done
cd "${HOME}"/march/ros2 || exit
rm "${DONE_FILE}"
source /opt/ros/foxy/setup.bash
source "${HOME}"/march/ros2/install/local_setup.bash

if ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; then
  echo "Ros1 is not done yet, sleeping for ${SLEEP_TIME}. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  sleep $SLEEP_TIME;
fi

colcon build
touch "${DONE_FILE}"

