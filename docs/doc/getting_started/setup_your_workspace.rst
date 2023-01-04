
.. _setup-your-workspace-label:

Building your workspace
=======================
.. inclusion-introduction-start

This tutorial will help you build the and ROS2 Foxy workspace with all packages needed to run the |march|.
You should have followed :ref:`install_ros_and_march-label` before you can build your workspace.

.. inclusion-introduction-end

This tutorial will walk you through how to setup your workspace such that you can:
1. Build ROS2
2. Launch ROS2

.. note:: You will have to perform most steps multiple times throughout the development process.
    In the sections :ref:`march_aliases-label` you can find some convenient aliases.

Building ROS2
^^^^^^^^^^^^^
Before you can run the code you should build it, to create/update the executable used to run the code.

To build the MARCH ROS2 packages, you have to source your ROS2 Foxy installation and then invoke colcon:

.. code:: bash

    source /opt/ros/foxy/local_setup.bash
    cd ~/march/ros2
    colcon build

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
In the march repo there is a bash file containing shortcuts to easily build and run the code, and some other utilities.
To use these aliases in the terminal, the file should be sourced in the ~/.bashrc file, as shown below.

.. code-block:: bash

    source ~/march/march_aliases.sh

.. code:: bash

    # Source ROS distribution
    alias sfox='source /opt/ros/foxy/local_setup.bash'
    alias svenv='source ~/march/.venv_march/bin/activate'

    # Source MARCH packages
    alias sros2='source ~/march/ros2/install/local_setup.bash'

    # Navigate to MARCH directory
    alias cm='cd ~/march/'
    alias cm2='cd ~/march/ros2/'

    # Build and run ROS2
    alias march_build_ros2='sfox && cm2 && colcon build'
    alias march_run_ros2_sim='sfox && sros2 && ros2 launch march_launch march_simulation.launch.py'
    alias march_run_ros2_training='sfox && sros2 && ros2 launch march_launch march.launch.py'

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
    alias march_clean_ros2='confirm rm -rf ~/march/ros2/build ~/march/ros2/log ~/march/ros2/install'

    # To give errors colors in ros2
    export RCUTILS_COLORIZED_OUTPUT=1

    # Install dependencies
    alias install_dep_ros2='cm2 && sfox && rosdep install --from-paths src --ignore-src -y --rosdistro foxy'

    # Format code
    alias format_cpp='cm && python3 .scripts/run-clang-format.py -r ros1/src ros2/src --style=file -i'
    alias format_py='cm && black .'

    # Start Clion & PyCharm with no consol output
    alias pycharm_no_out='pycharm-professional > /dev/null 2> /dev/null & disown'
    alias clion_no_out='clion > /dev/null 2> /dev/null & disown'
