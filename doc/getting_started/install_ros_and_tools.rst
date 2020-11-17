
.. _install_ros_and_tools-label:

Install ROS and tools
=====================
.. inclusion-introduction-start

This tutorial will help you install ROS1 Melodic, ROS2 Foxy and the required buildtools and dependencies.

.. inclusion-introduction-end

Install ROS1 Melodic
^^^^^^^^^^^^^^^^^^^^
This tutorial is mostly copied from `Install ROS1 Melodic <https://wiki.ros.org/melodic/Installation/Ubuntu>`_.

1) First, install some tools:

..  code:: bash

    sudo apt install lsb-release gnupg2


2) The next step is to setup your sources.list:

.. code:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


3) Then, setup your keys:

.. code:: bash

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

4) If these steps went well, you should be able to install ROS Melodic:

.. code:: bash

    sudo apt update
    sudo apt install ros-melodic-desktop-full


5) Once you have ROS installed, make sure you have the most up to date packages:

.. code:: bash

  sudo rosdep init
  rosdep update  # No sudo
  sudo apt update
  sudo apt full-upgrade

6) Finally , for building our packages we use colcon. Install `colcon <https://github.com/colcon>`_:

.. code:: bash

  sudo apt-get install python3-colcon-common-extensions

..
    comment for now because this doesnt actually seem relevant on this page
    To install some optional tools that are run by GitLab Continuous Integration run:
    .. code:: bash

      pip install --user catkin_lint
      python2 -m pip install --user flake8 pep8-naming flake8-blind-except flake8-string-format flake8-builtins flake8-commas flake8-quotes flake8-print flake8-docstrings flake8-import-order
      sudo apt install clang-format

Install ROS2 Foxy
^^^^^^^^^^^^^^^^^
Installing ROS2 Foxy requires some more effort than installing ROS1 Melodic.
This tutorial is a slightly updated version of `Install ROS2 Foxy <https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/>`_.

1) The first step is to update the CMake version, as the default Ubuntu 18.04 doesn't comply with some ROS2 packages (This may take some time):

.. code:: bash

    cd ~/
    sudo apt update && sudo apt install -y build-essential libssl-dev wget
    wget https://github.com/Kitware/CMake/archive/v3.18.4.tar.gz && tar -zxvf v3.18.4.tar.gz && rm v3.18.4.tar.gz && cd CMake-3.18.4 && ./bootstrap && make && cd .. && mv CMake-3.18.4 .CMake-3.18.4
    cd .CMake-3.18.4 && sudo ln -s $(pwd) /usr/local/share/cmake-3.18 && sudo ln $(pwd)/bin/* /usr/local/bin

Verify CMake is correctly updated by running:

.. code:: bash

    cmake --version

The output should be exactly:

.. code::

    cmake version 3.18.4

    CMake suite maintained and supported by Kitware (kitware.com/cmake).

2) Then we have to make sure the right locale is set:

.. code:: bash

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

3) Now we can add the ROS2 apt repository:

.. code:: bash

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

4) And add the repository to the sources list

.. code:: bash

    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

5) The next step is to install the necessary development and ROS tools:

.. code:: bash

    sudo apt update && sudo apt install -y \
      build-essential \
      cmake \
      git \
      libbullet-dev \
      python3-colcon-common-extensions \
      python3-flake8 \
      python3-pip \
      python3-pytest-cov \
      python3-setuptools \
      python3-vcstool \
      wget
    # install some pip packages needed for testing
    python3 -m pip install -U \
      argcomplete \
      flake8-blind-except \
      flake8-builtins \
      flake8-class-newline \
      flake8-comprehensions \
      flake8-deprecated \
      flake8-docstrings \
      flake8-import-order \
      flake8-quotes \
      pytest-repeat \
      pytest-rerunfailures \
      pytest
    # install Fast-RTPS dependencies
    sudo apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev
    # install Cyclone DDS dependencies
    sudo apt install --no-install-recommends -y \
      libcunit1-dev

6) Now we can create a ROS2 Foxy workspace and retrieve the code:

.. code:: bash

    mkdir -p ~/ros2_foxy/src
    cd ~/ros2_foxy
    wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
    vcs import src < ros2.repos

7) Install dependencies using rosdep:

.. code:: bash

    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

8) Some additional dependencies have to be manually added:

.. code:: bash

    cd ~/ros2_foxy/src/ros2/
    git clone https://github.com/ros/xacro.git -b dashing-devel
    git clone https://github.com/ros/urdf_parser_py.git -b ros2
    git clone https://github.com/ros-controls/control_msgs.git -b foxy-devel

8) The final step is to build the ROS2 code. This may take a long time (> 1h):

.. code:: bash

    cd ~/ros2_foxy/
    # skip ros1_bridge package since that has to configured later
    colcon build --symlink-install --packages-skip ros1_bridge
