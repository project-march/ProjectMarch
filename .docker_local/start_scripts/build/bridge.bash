#!/bin/bash

SLEEP_TIME=120
SLEEP_COUNTER=0
DONE_FILE=install/.done
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

while ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; do
  if [ "$SLEEP_COUNTER" -gt "$SLEEP_TIME" ]; then
    SLEEP_COUNTER=0
    echo "Ros1 is not done yet, you get this message again in ${SLEEP_TIME} seconds. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  fi
  ((SLEEP_COUNTER++))
  sleep 1;
done;

source "${HOME}"/march/ros1/install/local_setup.bash

while ! [ -f "${HOME}"/march/ros2/"${DONE_FILE}" ]; do
  if [ "$SLEEP_COUNTER" -gt "$SLEEP_TIME" ]; then
    SLEEP_COUNTER=0
    echo "Ros2 is not done yet, you get this message again in ${SLEEP_TIME} seconds. (do 'touch ~/march/ros2/${DONE_FILE}' if it should be done)";
  fi
  ((SLEEP_COUNTER++))
  sleep 1;
done;

source "${HOME}"/march/ros2/install/local_setup.bash

cd "${HOME}"/ros1_bridge/ || exit
colcon build --packages-select ros1_bridge --cmake-force-configure
source install/local_setup.bash
exec ros2 run ros1_bridge dynamic_bridge --print-pairs
