.. _robot-model-label:

Robot model
===========

This tutorial lists the main components that allow you to visualize a robot model.
As the official documentation of these components is sufficient, we will only provide a brief description of each component.

tf2
^^^
`tf2 <http://wiki.ros.org/tf2>`_ is the second generation of the transform library. It produces so called `tf frames`.
These are the positions and orientations of all the parts in the `urdf`. The tf frames have input from the `robot_state_publisher`.
In case an IMU is used, the :ref:`march-data-collector-label` uses IMU data to send a transform of the ``imu_link`` to tf2.

The resulting `tf frames` are used by `rviz` for creating a visualisation. The `tf frames` are also used in the :ref:`march-data-collector-label`
to calculate the center of mass and capture point.

urdf
^^^^

`urdf <http://wiki.ros.org/urdf>`_ is an xml format used to describe robots. It contains of visuals as well as collisions.
`xacro <http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File>`_ is a macro language which allows you to use variables and other neat shortcuts in urdf files.
To gain practical experience, please see the `official tutorials <http://wiki.ros.org/urdf/Tutorials>`_.

joint_state_publisher
^^^^^^^^^^^^^^^^^^^^^
The `joint_state_publisher <http://wiki.ros.org/joint_state_publisher>`_ is a very useful package,
which reads your urdf and publishes a `JointState message <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_ for all your joints.


robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^
The `robot_state_publisher <http://wiki.ros.org/robot_state_publisher>`_ takes your urdf and the joint states, and calculates the respective tf frames.

rviz
^^^^
`rviz <http://wiki.ros.org/rviz>`_ is a 3D visualization tool which is capable of displaying information about your robot and its surroundings.
The visualization combines the urdf and the tf frames to show the robot model.
Please check the `official userguide <http://wiki.ros.org/rviz/UserGuide>`_ to start using it!

Example
^^^^^^^
An example launchfile has been provided in :codedir:`robot_model.launch <useful_ros_tools/launch/robot_model.launch>`.

Inspect the file to see how the different nodes are launched, and try it out yourself with the following command!

.. code::

  roslaunch march_tutorials robot_model.launch

If no robot shows up, make sure to add a ``RobotModel`` to the displays on the left.

If your robot shows up but appears in white/red, make sure the global option for ``Fixed Frame`` is set to ``world`` instead of ``map``.

See also
^^^^^^^^

`Difference between robot_state_publisher and joint_state_publisher
<https://answers.ros.org/question/275079/joint-state-publisher-and-robot-state-publisher>`_
