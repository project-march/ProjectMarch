#!/bin/bash

SLEEP_TIME=3s
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source "${HOME}"/march/ros1/install/local_setup.bash
source "${HOME}"/march/ros2/install/local_setup.bash
source "${HOME}"/ros1_bridge/install/local_setup.bash



while ! curl --silent --head "${ROS_MASTER_URI}"; do
  echo "Could not connect to roscore at ${ROS_MASTER_URI}, going to sleep for ${SLEEP_TIME}";
  sleep $SLEEP_TIME;
done;

cd "${HOME}"/ros1_bridge/ || exit
exec ros2 run ros1_bridge parameter_bridge
