# The reason this docker file is made and we did not use the generall one, is to save storage because it can use the
# same ros1 and ros2 layers. Second reason is that we have smaller intermediate steps. This helps as the normal one is
# very difficult to upload to gitlab due to its large dependency install steps.
# This makes use of multistage docker files. (see https://docs.docker.com/develop/develop-images/multistage-build/)
# date: 12-02-2021
# author: George Vegelien

# Uses the local ros2 image (if you don't have this, checkout registry.gitlab.com/project-march/march/local:ros2)
FROM ros2 as ros2_builder

# The reason ros1 is done after ros2 is because ros1 is the bigger image, so less packages need to be copied over.
# Note however that the standalone size seems smaller, but because there are less cached layers, it will take up more actual storage space.

# Uses the local ros1 image (if you don't have this, checkout registry.gitlab.com/project-march/march/local:ros1)
FROM ros1 as ros1_builder


# This sections copies all the ros2 relevant installed folders made in the local ros2 image.
COPY --from=ros2_builder /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2-latest.list
COPY --from=ros2_builder /opt/ros/foxy /opt/ros/foxy
COPY --from=ros2_builder /usr/local/sbin /usr/local/sbin
COPY --from=ros2_builder /usr/local/bin /usr/local/bin
COPY --from=ros2_builder /usr/sbin /usr/sbin
COPY --from=ros2_builder /usr/bin /usr/bin
COPY --from=ros2_builder /bin /bin
COPY --from=ros2_builder /sbin /sbin

# Set all the bridge environmental variables
ENV ROS_VERSION 2
ENV PKG_CONFIG_PATH /opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig
ENV ROS_PACKAGE_PATH /opt/ros/noetic/share
ENV ROS_ETC_DIR /opt/ros/noetic/etc/ros
ENV CMAKE_PREFIX_PATH /opt/ros/noetic
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PYTHONPATH /opt/ros/foxy/lib/python3.8/site-packages:/opt/ros/noetic/lib/python3/dist-packages
ENV ROS_MASTER_URI http://localhost:11311
ENV LD_LIBRARY_PATH /opt/ros/foxy/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu
ENV PATH /opt/ros/foxy/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROS_LOCALHOST_ONLY 0
ENV ROS1_DISTRO noetic
ENV ROS_ROOT /opt/ros/noetic/share/ros
ENV ROS2_DISTRO foxy
ENV ROS_DISTRO=foxy

# Install the needed ros1 and ros2 packages for the bridge, and curl for the script ~/march/start_scripts/run/bridge.bash.
# See (https://github.com/osrf/docker_images/blob/d1e081089b3f7d8c118561b0f39998e7163a5f0a/ros/foxy/ubuntu/focal/ros1-bridge/Dockerfile)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm=1.15.14-1* \
    ros-foxy-ros1-bridge=0.9.6-1* \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Installs all the ros2 dependencies that are new and/or not copied over by copying all the folders specified above.
COPY ros2/src /march/src
RUN apt-get update && rosdep update --rosdistro foxy && rosdep install --rosdistro foxy -y --from-paths /march/src --ignore-src
# Remove march files to save space
RUN rm -rf /march
