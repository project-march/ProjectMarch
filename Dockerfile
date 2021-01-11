FROM registry.gitlab.com/project-march/march:base

# Update rosdep
RUN rosdep update && apt update

# Add project to Docker container
ADD . /projects
WORKDIR /projects

# Install Python dependencies
RUN python3 -m pip install -r requirements.pip

# Install ROS 1 rosdep dependencies
RUN bash -c "source /opt/ros/noetic/local_setup.bash && rosdep install -y --from-paths ros1/src --rosdistro noetic --ignore-src"

# Install ROS 2 rosdep dependencies
RUN bash -c "source /opt/ros/foxy/local_setup.bash && rosdep install -y --from-paths ros2/src --rosdistro foxy --ignore-src"

# Remove project files from container
RUN rm -rf /projects

# Remove apt cache to reduce image size
WORKDIR /root
RUN rm -rf /var/lib/apt/lists/* && rm -rf /var/cache/apt/archives && apt-get clean
