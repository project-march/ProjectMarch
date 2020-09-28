FROM ros:melodic

# Add project to Docker container
ADD . /projects
WORKDIR /projects

# Install build tools
RUN apt update && apt upgrade -y && apt install python-rosdep python-rosinstall python-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python-pip python-catkin-pkg python-catkin-lint clang-tidy clang -y

# Install Python linters
RUN python2 -m pip install flake8 pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order mock autopep8

# Install rosdep dependencies
RUN rosdep install -y --from-paths src --ignore-src && pip install numpy_ringbuffer pyqtgraph && apt install ros-melodic-gazebo-ros-control -y

# Remove project files from container
RUN rm -rf /projects

RUN rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean
