#!/usr/bin/env bash
sudo apt-get update

sudo apt-get install -y python3-colcon-common-extensions python3-pip

mkdir -p ~/march_ws
cd ~/march_ws

# Download march repo
git clone git@gitlab.com:project-march/march.git

# Install dependencies
cd ~/march_ws/march
rosdep install -y --from-paths src --ignore-src

colcon build
source install/setup.bash
m
