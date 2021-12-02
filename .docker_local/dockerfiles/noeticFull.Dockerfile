# See https://hub.docker.com/_/ros
# date: 12-02-2021
# author: George Vegelien

FROM ros:noetic

# Because we build this image in a non-interactive environment, we should notify Debian that
# we cannot respond to required interaction. This will assume the default values in case
# an interactive window pops up.
ARG DEBIAN_FRONTEND=noninteractive

# To install graphical tools such as rqt_graph.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    && rm -rf /var/lib/apt/lists/*

# Build tools
# We need pip to install certain python dependencies
# colcon-common-extensions is used for building and testing ROS1
# clang is used to compile C++ code
# clang-tidy is used in a different pipeline stage
# We need git and git-lfs to get repo information from
# inside the container.
#
# The `rosdep`, `rosinstall`, `wstool` and `build-essential` packages
# are required for building ROS1, see https://wiki.ros.org/noetic/Installation/Ubuntu
RUN apt update && apt install -y python3-pip python3-colcon-common-extensions clang clang-tidy git git-lfs python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# This one is seperated for space caching reasons, if it is left out it wil be installed by rosdep install ...
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Non-ROS Python dependencies
COPY requirements.txt /march/requirements.txt
RUN bash -c "python3 -m pip install -r /march/requirements.txt"

# Copy the project into the container to automatically detect all dependencies.
COPY ros1/src /march/src

# Install all ROS1 packages
# This runs the `rosdep` program, which automatically finds dependencies declared in package.xml files
# and installs them. See https://docs.ros.org/en/independent/api/rosdep/html/commands.html for
# command line reference.
RUN apt update && rosdep update --rosdistro noetic && rosdep install --rosdistro noetic -y --from-paths /march/src --ignore-src

# Remove march files to save space
RUN rm -rf /march
