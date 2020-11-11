#!/usr/bin/env bash

# Update CMake version
sudo apt update && sudo apt install -y build-essential libssl-dev wget
wget https://github.com/Kitware/CMake/archive/v3.18.4.tar.gz && tar -zxvf v3.18.4.tar.gz && rm v3.18.4.tar.gz && cd CMake-3.18.4 && ./bootstrap && make
sudo make install && cd .. && rm -rf CMake-3.18.4

# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install development tools and ROS tools
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# Create workspace and retrieve code
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos

# Add additional dependencies
cd ~/ros2_foxy/src/ros2/
git clone https://github.com/ros/xacro/tree/dashing-devel
git clone https://github.com/ros/urdf_parser_py/tree/ros2
git clone https://github.com/ros-controls/control_msgs

# Build ROS2 code
colcon build --symlink-install --packages-skip ros1_bridge
