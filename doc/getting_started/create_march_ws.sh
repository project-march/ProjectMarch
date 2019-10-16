#!/usr/bin/env bash
sudo apt-get update

sudo apt-get install -y python3-colcon-common-extensions python3-pip
pip3 install -U setuptools catkin_lint catkin_tools_document

mkdir -p ~/march_ws/src
cd ~/march_ws

wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall
wstool update -t src

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

colcon build
source install/setup.bash
