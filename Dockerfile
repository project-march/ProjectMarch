FROM ros:foxy-ros1-bridge-focal

# Install ROS Noetic and Foxy base packages and build tools
RUN apt update && apt upgrade -y && apt install -y apt-utils && apt install -y ros-noetic-ros-base ros-foxy-ros-base && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git git-lfs libbullet-dev python3-flake8 python3-pytest-cov python3-setuptools python3-vcstool && apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev && python3 -m pip install mock argcomplete pytest-repeat pytest-rerunfailures pytest flakehell

# Set the workdirectory to /projects
WORKDIR /projects

# Because Dockers COPY command cannot just copy only the "package.xml" files, it is useful to hardcode some of the ROS packages in this stage.
# The installation of ROS packages can than be cached for the significant part.
# This list is generated with `.scripts/package_list.sh`.
RUN apt update && apt install -y python3-scipy ros-noetic-gazebo-ros ros-noetic-moveit-ros-benchmarks ros-noetic-trajectory-msgs ros-noetic-urdf ros-noetic-message-generation ros-noetic-code-coverage ros-noetic-tf2-geometry-msgs ros-noetic-xacro ros-noetic-geometry-msgs ros-noetic-rqt-robot-monitor ros-noetic-hardware-interface python3-numpy libyaml-cpp-dev ros-noetic-controller-manager ros-noetic-soem ros-noetic-moveit-setup-assistant ros-noetic-gazebo-ros-control gazebo11 ros-noetic-rospy ros-noetic-moveit-kinematics ros-noetic-robot-state-publisher ros-noetic-moveit-ros-warehouse ros-noetic-moveit-planners-ompl ros-noetic-visualization-msgs ros-noetic-rviz ros-noetic-sound-play python3-mock ros-noetic-joint-state-controller ros-noetic-actionlib ros-noetic-urdfdom-py ros-noetic-diagnostic-updater ros-noetic-realtime-tools ros-noetic-message-runtime ros-noetic-std-msgs ros-noetic-joy python3-parameterized ros-noetic-controller-interface ros-noetic-roslib ros-noetic-roscpp ros-noetic-gazebo-msgs ros-noetic-moveit-ros-move-group ros-noetic-rqt-gui ros-noetic-rosunit ros-noetic-moveit-commander ros-noetic-warehouse-ros-mongo ros-noetic-actionlib-msgs ros-noetic-rqt-gui-py ros-noetic-control-msgs ros-noetic-joint-state-publisher ros-noetic-pluginlib ros-noetic-tf2-ros ros-noetic-tf ros-noetic-moveit-ros-visualization ros-noetic-rostest ros-noetic-moveit-fake-controller-manager ros-noetic-joint-trajectory-controller ros-noetic-diagnostic-aggregator ros-noetic-joint-limits-interface python3-setuptools ros-noetic-catkin ros-noetic-rosserial-python ros-noetic-dynamic-reconfigure ros-noetic-sensor-msgs ros-foxy-rqt-gui python3-setuptools ros-foxy-rosidl-default-runtime ros-foxy-geometry-msgs ros-foxy-control-msgs ros-foxy-urdfdom-py ros-foxy-ament-cmake ros-foxy-gazebo-msgs ros-foxy-ament-lint-common ros-foxy-builtin-interfaces ros-foxy-actionlib-msgs ros-foxy-rclcpp ros-foxy-trajectory-msgs ros-foxy-rosgraph-msgs ros-foxy-rqt-gui-py ros-foxy-ament-lint-auto ros-foxy-xacro ros-foxy-std-msgs python3-pytest ros-foxy-rosidl-default-generators ros-foxy-ament-cmake-gtest ros-foxy-sensor-msgs ros-foxy-rclpy

# Install flakehell linter dependencies
COPY pyproject.toml .flake-baseline /projects/
RUN bash -c "python3 -m pip install -r <(python3 -m flakehell missed) && rm -f /projects/pyproject.toml"

# Install Python required runtime dependencies and test dependencies
COPY requirements.pip /projects/
RUN python3 -m pip install -r requirements.pip && rm -f /projects/requirements.pip

# Install ROS 1 rosdep dependencies. This will only install packages that are not defined above.
COPY ros1 /projects/
RUN bash -c "source /opt/ros/noetic/local_setup.bash && rosdep update && apt update && rosdep install -y --from-paths /projects/src --rosdistro noetic --ignore-src && rm -rf /projects && rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean"

# Install ROS 2 rosdep dependencies. This will only install packages that are not defined above.
COPY ros2 /projects/
RUN bash -c "source /opt/ros/foxy/local_setup.bash && rosdep update && apt update && rosdep install -y --from-paths /projects/src --rosdistro foxy --ignore-src && rm -rf /projects && rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean"

# Set working directory
WORKDIR /root
