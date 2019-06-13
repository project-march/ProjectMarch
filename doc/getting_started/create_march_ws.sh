#!/usr/bin/env bash

sudo apt-get install python-catkin-tools -y
pip install --user catkin_lint
pip install --user catkin_tools_document

source /opt/ros/kinetic/setup.bash
mkdir -p march_ws/src
cd march_ws
catkin init --workspace .
catkin build
wstool init src https://gist.githubusercontent.com/Ishadijcks/ede0d44f04fd5c0abaea839e5b1bc950/raw/58719ef7506be5e7e5b80b502250bd3273f56dfe/march_ws.rosinstall
wstool update -t src

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
source devel/setup.bash