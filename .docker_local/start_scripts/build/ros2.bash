#!/bin/bash

SLEEP_TIME=30
SLEEP_COUNTER=0
DONE_FILE=install/.done
cd "${HOME}"/march/ros2 || exit
rm "${DONE_FILE}"
source /opt/ros/foxy/setup.bash

while ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; do
  if [ $SLEEP_COUNTER == $SLEEP_TIME ]; then
    SLEEP_COUNTER=0
    echo "Ros1 is not done yet, you get this message again in ${SLEEP_TIME} seconds. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  fi
  ((SLEEP_COUNTER++))
  sleep 1;
done;

colcon build
touch "${DONE_FILE}"

