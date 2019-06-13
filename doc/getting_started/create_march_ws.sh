#!/usr/bin/env bash

sudo apt-get install python-catkin-tools -y
pip install --user catkin_lint
pip install --user catkin_tools_document

source /opt/ros/kinetic/setup.bash
mkdir -p march_ws/src
cd march_ws
catkin init --workspace .
catkin build
wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall
wstool update -t src

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
source devel/setup.bash