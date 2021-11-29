FROM ros:foxy-ros1-bridge

# Because we build this image in a non-interactive environment, we should notify Debian that
# we cannot respond to required interaction. This will assume the default values in case
# an interactive window pops up.
ARG DEBIAN_FRONTEND=noninteractive

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

# Non-ROS Python dependencies
COPY requirements.txt /march/requirements.txt
RUN bash -c "python3 -m pip install -r /march/requirements.txt"

# Copy the project into the container to automatically detect all dependencies.
COPY ros1/src /march/ros1/src

# Copy the project into the container to automatically detect all dependencies.
COPY ros2/src /march/ros2/src


# Install all ROS1 packages
# This runs the `rosdep` program, which automatically finds dependencies declared in package.xml files
# and installs them. See https://docs.ros.org/en/independent/api/rosdep/html/commands.html for
# command line reference.
RUN apt update && rosdep update --rosdistro noetic && rosdep install --rosdistro noetic -y --from-paths /march/ros1/src --ignore-src


# Install all ROS2 packages
# This runs the `rosdep` program, which automatically finds dependencies declared in package.xml files
# and installs them. See https://docs.ros.org/en/independent/api/rosdep/html/commands.html for
# command line reference.
RUN apt update && rosdep update --rosdistro foxy && rosdep install --rosdistro foxy -y --from-paths /march/ros2/src --ignore-src

# Remove march files to save space
RUN rm -rf /march


# Remove march files to save space
RUN rm -rf /march
