
.. _setup-your-workspace-label:

Building your workspace
=======================
.. inclusion-introduction-start

This tutorial will help you build the  ROS1 Noetic and ROS2 Foxy workspace with all packages needed to run the |march|.
You should have followed :ref:`install_ros_and_march-label` before you can build your workspace.

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
    source ~/march/ros1/install/local_setup.bash
    source ~/march/ros2/install/local_setup.bash
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
    ros2 launch march_launch march_simulation.launch.py

.. _march_aliases-label:

Convenient aliases
^^^^^^^^^^^^^^^^^^
These aliases provide shortcuts to easily build and run the code, and some other utilities.
It is recommended you add these aliases to a separate the a file and link to those.
First link to the march aliases file by adding the below code to your ~/.bashrc file.

.. code-block:: bash

    if [ -f ~/.march_bash_aliases ]; then
        . ~/.march_bash_aliases
    fi

Secondly add a file named ~/.march_bash_aliases, and paste the code below in that file.

.. note:: You can also add the code below directly into your ~/.bashrc file, and skip the step above.
    However this is not recommended as it is less neat.

.. code:: bash

    # Source ROS distribution
    alias snoe='source /opt/ros/noetic/local_setup.bash'
    alias sfox='source /opt/ros/foxy/local_setup.bash'
    alias svenv='source ~/march/.venv_march/bin/activate'

    # Source MARCH packages
    alias sros1='source ~/march/ros1/install/local_setup.bash'
    alias sros2='source ~/march/ros2/install/local_setup.bash'

    # Navigate to MARCH directory
    alias cm1='cd ~/march/ros1/'
    alias cm='cd ~/march/'
    alias cm2='cd ~/march/ros2/'

    # Build and run ROS1
    alias march_build_ros1='snoe && cm1 && colcon build'
    alias march_run_ros1_sim='snoe && sros1 && roslaunch march_launch march_simulation.launch'
    alias march_run_ros1_airgait='snoe && sros1 && roslaunch march_launch march.launch if_name:=enp2s0f0'
    alias march_run_ros1_groundgait='march_run_ros1_airgait ground_gait:=true gain_tuning:=groundgait'
    alias march_run_ros1_training='march_run_ros1_groundgait gain_tuning:=training'

    # Build and run ROS2
    alias march_build_ros2='sfox && cm2 && colcon build'
    alias march_run_ros2_sim='sfox && sros2 && ros2 launch march_launch march_simulation.launch.py'
    alias march_run_ros2_training='sfox && sros2 && ros2 launch march_launch march.launch.py'

    # Build and run the bridge
    alias march_build_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && colcon build --packages-select ros1_bridge --cmake-force-configure && source install/local_setup.bash && ros2 run ros1_bridge dynamic_bridge --print-pairs'
    alias march_run_bridge='snoe && sfox && sros1 && sros2 && cd ~/ros1_bridge && source install/local_setup.bash && export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge parameter_bridge'

    # Clean march builds
    # script to ask for confirmation before cleaning ros
    confirm() {
        echo -n "Do you want to run $*? [N/y] "
        read -N 1 REPLY
        echo
        if test "$REPLY" = "y" -o "$REPLY" = "Y"; then
            "$@"
        else
            echo "Cancelled by user"
        fi
    }
    alias march_clean_ros1='confirm rm -rf ~/march/ros1/build ~/march/ros1/log ~/march/ros1/install'
    alias march_clean_ros2='confirm rm -rf ~/march/ros2/build ~/march/ros2/log ~/march/ros2/install'
    alias march_clean_bridge='confirm rm -rf ~/ros1_bridge/build ~/ros1_bridge/log ~/ros1_bridge/install'
    alias march_clean_all='march_clean_ros1 && march_clean_ros2 && march_clean_bridge'

    # To give errors colors in ros2
    export RCUTILS_COLORIZED_OUTPUT=1

    # Install dependencies
    alias install_dep_ros1='cm1 && snoe && rosdep install --from-paths src --ignore-src -y --rosdistro noetic'
    alias install_dep_ros2='cm2 && sfox && rosdep install --from-paths src --ignore-src -y --rosdistro foxy'

    # Format code
    alias format_cpp='cm && python3 .scripts/run-clang-format.py -r ros1/src ros2/src --style=file -i'
    alias format_py='cm && black .'

    # Start Clion & PyCharm with no consol output
    alias pycharm_no_out='pycharm-professional > /dev/null 2> /dev/null & disown'
    alias clion_no_out='clion > /dev/null 2> /dev/null & disown'
