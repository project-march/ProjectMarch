.. _march-launch-label:

march_launch
============
The march_launch package is a collection of launch files which are used to start the exoskeleton.

The launch files are split up into different launch files, where higher level launch files can include other launch
files to launch a set of nodes or set parameters. The central high level launch file is
:march:`march.launch <march_launch/launch/march.launch>`. This launch file has many arguments to launch different
configurations of nodes. To see all possible arguments use:

.. code::

  roslaunch --ros-args march_launch march.launch

The most important one is ``configuration``, which decides whether to launch the simulation or the hardware.
Since these configurations are used most of the time in a predefined configuration, there exist two more high level
launch files: :march:`march_simulation.launch <march_launch/launch/march_simulation.launch>` and
:march:`march_headless.launch <march_launch/launch/march_headless.launch>`. The first is configured for the simulation
and the second for the hardware, without any GUI applications. These launch files simply include the ``march.launch``
file and set some default parameters. The default parameters for ``march.launch`` are set to launch the hardware
with RQT input device, so you can use this on the exoskeleton to launch:

.. code::

  roslaunch march_launch march.launch

For more information on how to launch the exoskeleton, see :ref:`how-to-airgait-label`.
