
.. install_ros_and_march-label:

Install ROS and March
=====================
.. inclusion-introduction-start

This tutorial will help you install ROS1 Noetic, ROS2 Foxy, the MARCH repository, the required buildtools and dependencies.
It is recommended to follow this tutorial on Ubuntu 20.04 (Focal), other operating systems have not been tested.

.. inclusion-introduction-end

Download the MARCH source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
First, make sure both git and git lfs are installed:

.. code:: bash

    sudo apt install git git-lfs

The next step is to download the source code, you can either use ssh (recommended):

.. code:: bash

    cd ~/
    git clone git@gitlab.com:project-march/march.git --recurse-submodules
    cd march/
    git lfs install


Or use https:

.. code:: bash

    cd ~/
    git clone https://gitlab.com/project-march/march.git --recurse-submodules
    cd march/
    git lfs install


Install ROS1 Noetic
^^^^^^^^^^^^^^^^^^^

This tutorial is mostly copied from `Install ROS1 Noetic <https://wiki.ros.org/noetic/Installation/Ubuntu>`_.

1. First, install some tools:

..  code:: bash

    sudo apt install lsb-release gnupg2


2. The next step is to setup your sources.list:

.. code:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

3. Then, setup your keys:

.. code:: bash

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

4. If these steps went well, you should be able to install colcon and ROS Noetic:

.. code:: bash

    sudo apt update
    sudo apt install python3-colcon-common-extensions
    sudo apt install ros-noetic-desktop-full


5. Once you have ROS installed, make sure you have the most up to date packages:

.. code:: bash

  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update  # No sudo
  sudo apt update
  sudo apt full-upgrade

6. Finally, the following will install any ROS1 Noetic package dependencies not already in your workspace:

.. code:: bash

  sudo apt update
  source /opt/ros/noetic/local_setup.bash
  cd ~/march/ros1/
  rosdep install --rosdistro noetic -y --from-paths src --ignore-src


Install ROS2 Foxy
^^^^^^^^^^^^^^^^^
Installing ROS2 Foxy requires some more effort than installing ROS1 Noetic.
This tutorial is a slightly updated version of `Install ROS2 Foxy <https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/>`_.

1. The first step is to make sure the right locale is set:

.. code:: bash

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

2. Now we can add the ROS2 apt repository:

.. code:: bash

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

3. And add the repository to the sources list

.. code:: bash

    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

4. Update your apt repository caches after setting up the repositories.

.. code:: bash

    sudo apt update
    sudo apt install ros-foxy-desktop

5. Install argcomplete (optional), this is used for autocompletion on command line

.. code:: bash

    sudo apt install -y python3-pip
    pip3 install -U argcomplete

6. Install dependencies using rosdep:

.. code:: bash

    source /opt/ros/foxy/local_setup.bash
    cd ~/march/ros2
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y

Install ROS1/ROS2 bridge
^^^^^^^^^^^^^^^^^^^^^^^^
1. Clone the bridge repository

.. code:: bash

    cd ~/
    git clone https://github.com/ros2/ros1_bridge.git

Install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Some additional python dependencies have to be installed using pip:

.. code:: bash

    python3 -m pip install -r ~/march/requirements.pip

Install RealSense dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are planning to also use the Intel Realsense camera, you should also install the necessary packages for this:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages.

