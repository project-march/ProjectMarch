Getting Started
===============

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

Install `catkin <http://wiki.ros.org/catkin>`_ the ROS build system: ::

  sudo apt-get install ros-kinetic-catkin python-catkin-tools

Create A Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a `catkin <http://wiki.ros.org/catkin>`_ workspace setup: ::

  mkdir -p ~/march_ws/src

Download the march source code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TODO

Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  cd ~/march_ws/src
  rosdep install -y --from-paths . --ignore-src --rosdistro kinetic

The next command will configure your catkin workspace: ::

  cd ~/march_ws
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Debug
  catkin build

Source the catkin workspace: ::

  source ~/march_ws/devel/setup.bash

Optional: add the previous command to your ``.bashrc``: ::

   echo 'source ~/march_ws/devel/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   catkin workspace at a time, but we recommend it for simplicity.

Next Step
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TODO Rviz quickstart
`Visualize a robot with the interactive motion planning plugin for RViz <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
