
.. _install_ros_and_march-label:

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

To install ROS1, just follow: `Install ROS1 Noetic <https://wiki.ros.org/noetic/Installation/Ubuntu>`_.

Once you have ROS installed, make sure you have the most up to date packages:

.. code:: bash

  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update  # No sudo
  sudo apt update
  sudo apt full-upgrade

Install dependencies using rosdep:

.. code:: bash

  sudo apt update
  source /opt/ros/noetic/local_setup.bash
  cd ~/march/ros1/
  rosdep install --rosdistro noetic -y --from-paths src --ignore-src


Install ROS2 Foxy
^^^^^^^^^^^^^^^^^
To install ROS2, just follow: `Install ROS2 Foxy <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html  />`_.

Install dependencies using rosdep:

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

