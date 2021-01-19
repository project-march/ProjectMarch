FROM ros:foxy-ros1-bridge-focal

# Install ROS Noetic and Foxy base packages
RUN apt update && apt upgrade -y && apt install -y apt-utils && apt install -y ros-noetic-ros-base ros-foxy-ros-base

# Install build tools
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git git-lfs libbullet-dev python3-flake8 python3-pytest-cov python3-setuptools python3-vcstool && apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev

# Install Python required runtime dependencies and test dependencies
RUN python3 -m pip install mock argcomplete pytest-repeat pytest-rerunfailures pytest flakehell

# Update rosdep
RUN rosdep update

# Add project to Docker container
ADD . /projects
WORKDIR /projects

# Install Python linter plugins & dependencies
RUN python3 -m pip install -r requirements.pip && bash -c "python3 -m pip install -r <(python3 -m flakehell missed)"

# Install ROS 1 rosdep dependencies
RUN bash -c "source /opt/ros/noetic/local_setup.bash && rosdep install -y --from-paths ros1/src --rosdistro noetic --ignore-src"

# Install ROS 2 rosdep dependencies
RUN bash -c "source /opt/ros/foxy/local_setup.bash && rosdep install -y --from-paths ros2/src --rosdistro foxy --ignore-src"

# Remove project files from container
RUN rm -rf /projects

# Remove apt cache to reduce image size
WORKDIR /root
RUN rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean
