#!/usr/bin/env bash
sudo apt-get update

sudo apt-get install -y python3-colcon-common-extensions python3-pip

mkdir -p ~/march_ws/src
cd ~/march_ws || exit

wstool init src https://raw.githubusercontent.com/project-march/tutorials/develop/doc/getting_started/.rosinstall
wstool update -t src

# Install dependencies
rosdep install -y --from-paths src --ignore-src

colcon build
source install/setup.bash
