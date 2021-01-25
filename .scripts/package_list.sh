rosdep keys --from-paths ros1/src --rosdistro noetic --ignore-src | xargs rosdep resolve --rosdistro noetic | grep -v "^#"
rosdep keys --from-paths ros2/src --rosdistro foxy --ignore-src | xargs rosdep resolve --rosdistro foxy | grep -v "^#"
