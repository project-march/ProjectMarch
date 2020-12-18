From ros:foxy-ros1-bridge-focal

# Add project to Docker container
ADD . /projects
WORKDIR /projects

# Install ROS Noetic and Foxy base packages
RUN apt update && apt install -y ros-noetic-ros-base ros-foxy-ros-base

# Install build tools
RUN apt update && apt upgrade -y && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git libbullet-dev python3-flake8 python3-pytest-cov python3-setuptools python3-vcstool && sudo apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev

# Install Python linters
RUN python3 -m pip install pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order mock autopep8 pydocstyle argcomplete flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings pytest-repeat pytest-rerunfailures pytest

# Install rosdep dependencies
RUN rosdep install -y --from-paths ros1/src --ignore-src && pip install -r requirements.pip

# Install additional dependencies that are not available the standard way
RUN mkdir /ros2_foxy && cd /ros2_foxy && vcs import src < ros2_dependencies.repos && source /opt/ros/foxy/local_setup.sh && colcon build

# Remove project files from container
RUN rm -rf /projects

# Update CMake to version 3.18
WORKDIR /root
RUN rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean
