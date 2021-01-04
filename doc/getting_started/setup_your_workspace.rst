
.. _setup-your-workspace-label:

Building your workspace
=======================
.. inclusion-introduction-start

This tutorial will help you build the  ROS1 Noetic and ROS2 Foxy workspace with all packages needed to run the |march|.
You should have followed :ref: `install_ros_and_march-label` before you can build your workspace.

.. inclusion-introduction-end


This tutorial will walk you through how to setup your workspace such that you can:
1. Build ROS1
2. Build ROS2
3. Build the bridge that connects them.
4. Launch ROS1
5. Launch the bridge
6. Launch ROS2

.. note:: You will have to perform most steps multiple times throughout the development process.
    In the sections :ref:`march_aliases-label` you can find some convenient aliases.


Building the code
-----------------
The ROS1 MARCH packages, ROS2 MARCH packages, and the bridge should all be built in a **separate terminal**.
Note that you should only build the bridge after the ROS1 and ROS2 MARCH packages have been built

Building ROS1
^^^^^^^^^^^^^

To build the MARCH ROS1 packages, you have to source your ROS1 Noetic installation and then invoke colcon:


.. code:: bash

    source /opt/ros/noetic/local_setup.bash
    cd ~/march/ros1
    colcon build

Building ROS2
^^^^^^^^^^^^^

To build the MARCH ROS2 packages, you have to source your ROS2 Foxy installation and then invoke colcon:

.. code:: bash

    source /opt/ros/foxy/local_setup.bash
    cd ~/march/ros2
    colcon build

Building the bridge
^^^^^^^^^^^^^^^^^^^

Building the bridge usually only needs to be done once.
Rebuilding the bridge is only necessary when new messages are added in both ROS1 and ROS2 which need to be bridged.
To build the bridge, you have to source *all* your ROS files,
and then use colcon to build the 'ros1_bridge' package:

.. code:: bash

    source /opt/ros/noetic/local_setup.bash
    source /opt/ros/foxy/local_setup.bash
    source ~/march/ros1/install/local_setup.bash
    source ~/march/ros2/install/local_setup.bash
    cd ~/ros1_bridge
    colcon build --packages-select ros1_bridge --cmake-force-configure

If you want to see the messages that are mapped and verify the bridge was built correctly run:

.. code:: bash

    ros2 run ros1_bridge dynamic_bridge --print-pairs

Now everything is ready to run ROS1, ROS2 and the bridge.

Running the code
----------------
ROS1 MARCH packages, ROS2 MARCH packages, and the bridge should all be run in a **separate terminal**.
The order of startup is:
1. ROS1
2. The bridge
3. ROS2

Run ROS1
^^^^^^^^

In order to run ROS1, you have to source both ROS1 Noetic and the ROS1 MARCH packages.

.. code:: bash

    source /opt/ros/noetic/local_setup.bash
    source ~/march/ros1/install/local_setup.bash
    roslaunch march_launch march_simulation.launch

Run the bridge
^^^^^^^^^^^^^^

In order to run the bridge, you have to source ROS1 and ROS2.

.. code:: bash

    source /opt/ros/noetic/local_setup.bash
    source /opt/ros/foxy/local_setup.bash
    cd ~/ros1_bridge
    source install/local_setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Run ROS2
^^^^^^^^

In order to run ROS2, you have to source both ROS2 Foxy and the ROS2 MARCH packages.

.. code:: bash

    source /opt/ros/foxy/local_setup.bash
    source ~/march/ros2/install/local_setup.bash
    ros2 launch march_launch march_ros2_simulation.launch.py

.. _march_aliases-label:

Convenient aliases
^^^^^^^^^^^^^^^^^^
These aliases provide shortcuts to easily build and run the code. It is recommended you add them to your ~/.bashrc file.

.. code:: bash

    alias snoe='source /opt/ros/noetic/local_setup.bash'
    alias sfox='source /opt/ros/foxy/local_setup.bash'

    alias sros1='source ~/march/ros1/install/local_setup.bash'
    alias sros2='source ~/march/ros2/install/local_setup.bash'

    alias march_build_ros1='bash -i -c "snoe && cd ~/march/ros1 && colcon build"'
    alias march_run_ros1='bash -i -c "snoe && sros1 && roslaunch march_launch march_simulation.launch gait_directory:=test_versions-vi"'

    alias march_build_ros2='bash -i -c "sfox && cd ~/march/ros2 && colcon build"'
    alias march_run_ros2='bash -i -c "sfox && sros2 && ros2 launch march_launch march_ros2_simulation.launch.py"'

    alias march_build_bridge='bash -i -c "snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && colcon build --packages-select ros1_bridge --cmake-force-configure && source install/local_setup.bash && ros2 run ros1_bridge dynamic_bridge --print-pairs"'
    alias march_run_bridge='bash -i -c "snoe && sfox && cd ~/ros1_bridge && source install/local_setup.bash && export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"'