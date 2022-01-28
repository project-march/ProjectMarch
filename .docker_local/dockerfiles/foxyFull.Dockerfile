# See https://hub.docker.com/_/ros
# date: 12-02-2021
# author: George Vegelien

FROM ros:foxy

# Because we build this image in a non-interactive environment, we should notify Debian that
# we cannot respond to required interaction. This will assume the default values in case
# an interactive window pops up.
ARG DEBIAN_FRONTEND=noninteractive

# To install graphical tools such as rqt_graph.
# The 'ninja-build' package is for generating a json-database needed for static analysis.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Build tools
# We need pip to install certain python dependencies
# clang is used to compile C++ code
# clang-tidy is used in a different pipeline stage
# We need git and git-lfs to get repo information from
# inside the container.
RUN apt update && apt install -y python3-pip clang clang-tidy git git-lfs

# Non-ROS Python dependencies
COPY requirements.txt /march/requirements.txt
RUN bash -c "python3 -m pip install -r /march/requirements.txt"

# Copy the project into the container to automatically detect all dependencies.
COPY ros2/src /march/src

# Install all ROS2 packages
# This runs the `rosdep` program, which automatically finds dependencies declared in package.xml files
# and installs them. See https://docs.ros.org/en/independent/api/rosdep/html/commands.html for
# command line reference.
RUN apt update && rosdep update --rosdistro foxy && rosdep install --rosdistro foxy -y --from-paths /march/src --ignore-src

# Remove march files to save space
RUN rm -rf /march
