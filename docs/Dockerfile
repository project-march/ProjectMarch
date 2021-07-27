# Use ROS noetic as base image
FROM ros:noetic-ros-core

# Install build tools
RUN apt update && apt upgrade -y && apt install -y python3-rosdep python3-pip ros-noetic-rosdoc-lite python3-pygit2

# Install html-proofer
RUN apt install -y ruby-dev && gem update --system && gem --version && gem install html-proofer

# Install Python dependencies
RUN python3 -m pip install sphinx-rtd-theme
