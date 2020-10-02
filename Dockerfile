FROM ros:melodic-ros-core

# Install build tools
RUN apt update && apt upgrade -y && apt install -y python-rosdep python-pip ros-melodic-rosdoc-lite python-pygit2

# Install html-proofer
RUN apt install -y ruby-dev && gem update --system && gem --version && gem install html-proofer

# Install Python dependencies
RUN python2 -m pip install sphinx-rtd-theme
