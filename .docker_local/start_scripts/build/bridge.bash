#!/bin/bash

SLEEP_TIME=30s
DONE_FILE=install/.done
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

if ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; then
  echo "Ros1 is not done yet, sleeping for ${SLEEP_TIME}. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  sleep $SLEEP_TIME;
fi

source "${HOME}"/march/ros1/install/local_setup.bash

if ! [ -f "${HOME}"/march/ros2/"${DONE_FILE}" ]; then
  echo "Ros2 is not done yet, sleeping for ${SLEEP_TIME}. (do 'touch ~/march/ros2/${DONE_FILE}' if it should be done)";
  sleep $SLEEP_TIME;
fi

source "${HOME}"/march/ros2/install/local_setup.bash

cd "${HOME}"/ros1_bridge/ || exit
colcon build --packages-select ros1_bridge --cmake-force-configure
source install/local_setup.bash
exec ros2 run ros1_bridge dynamic_bridge --print-pairs
