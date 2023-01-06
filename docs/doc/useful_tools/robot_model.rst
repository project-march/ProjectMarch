.. _robot-model-label:

Robot model
===========

This tutorial lists the main components that allow you to visualize a robot model.
As the official documentation of these components is sufficient, we will only provide a brief description of each component.

tf2
^^^
`tf2 <https://wiki.ros.org/tf2>`_ is the second generation of the transform library. It produces so called `tf frames`.
These are the positions and orientations of all the parts in the `urdf`. The tf frames have input from the `robot_state_publisher`.
In case an IMU is used, the :ref:`march-data-collector-label` uses IMU data to send a transform of the ``imu_link`` to tf2.

The resulting `tf frames` are used by `rviz` for creating a visualization. The `tf frames` are also used in the :ref:`march-data-collector-label`
to calculate the center of mass and capture point.

When you select to unfix the exoskeleton in simulation a script in the :ref:`march-simulation-label` is used to produce
a transform that copies the movements of the exoskeleton with respect to the world. The movements are then also visible in `rviz`.

urdf
^^^^

`urdf <https://wiki.ros.org/urdf>`_ is an xml format used to describe robots. It contains of visuals as well as collisions.
`xacro <https://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File>`_ is a macro language which allows you to use variables and other neat shortcuts in urdf files.
To gain practical experience, please see the `official tutorials <https://wiki.ros.org/urdf/Tutorials>`_.

joint_state_publisher
^^^^^^^^^^^^^^^^^^^^^
The `joint_state_publisher <https://wiki.ros.org/joint_state_publisher>`_ is a very useful package,
which reads your urdf and publishes a `JointState message <https://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_ for all your joints.


robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^
The `robot_state_publisher <https://wiki.ros.org/robot_state_publisher>`_ takes your urdf and the joint states, and calculates the respective tf frames.

rviz
^^^^
`rviz <https://wiki.ros.org/rviz>`_ is a 3D visualization tool which is capable of displaying information about your robot and its surroundings.
The visualization combines the urdf and the tf frames to show the robot model.
Please check the `official userguide <https://wiki.ros.org/rviz/UserGuide>`_ to start using it!

See also
^^^^^^^^

`Difference between robot_state_publisher and joint_state_publisher
<https://answers.ros.org/question/275079/joint-state-publisher-and-robot-state-publisher>`_
