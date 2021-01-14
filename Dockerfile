FROM ros:foxy-ros1-bridge-focal

# Install ROS Noetic and Foxy base packages and build tools
RUN apt update && apt upgrade -y && apt install -y apt-utils && apt install -y ros-noetic-ros-base ros-foxy-ros-base && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git git-lfs libbullet-dev python3-flake8 python3-pytest-cov python3-setuptools python3-vcstool && apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev && python3 -m pip install mock argcomplete pytest-repeat pytest-rerunfailures pytest flakehell

# Set the workdirectory to /projects
WORKDIR /projects

# Install flakehell linter dependencies
COPY pyproject.toml .flake-baseline /projects/
RUN bash -c "python3 -m pip install -r <(python3 -m flakehell missed) && rm -f /projects/pyproject.toml"

# Install Python required runtime dependencies and test dependencies
COPY requirements.pip /projects/
RUN python3 -m pip install -r requirements.pip && rm -f /projects/requirements.pip

# Update rosdep
RUN rosdep init

# Install ROS 1 rosdep dependencies
COPY ros1 /projects/
RUN bash -c "source /opt/ros/noetic/local_setup.bash && rosdep update && apt update && rosdep install -y --from-paths /projects/src --rosdistro noetic --ignore-src && rm -rf /projects && rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean"

# Install ROS 2 rosdep dependencies
COPY ros2 /projects/
RUN bash -c "source /opt/ros/foxy/local_setup.bash && rosdep update && apt update && rosdep install -y --from-paths /projects/src --rosdistro foxy --ignore-src && rm -rf /projects && rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean"

# Set working directory
WORKDIR /root
