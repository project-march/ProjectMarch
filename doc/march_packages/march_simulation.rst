.. _march-simulation-label:

march_simulation
================

Overview
--------
The march simulation package launches `Gazebo <http://gazebosim.org/>`_ and spawns the :ref:`URDF <robot-model-label>` of the |march|.
Its goal is to provide an interface for our high-level code to call similar to that of the :ref:`march-hardware-interface-label`.

As we only use Gazebo for actuating joints, using it is very simple.
Please check :simulation:`march_world.launch <march_simulation/launch/march_world.launch>` to see how its used.

ROS Control
^^^^^^^^^^^
To integrate Gazebo with ROS Control, we highly recommend checking out the `Gazebo ros control tutorial <http://gazebosim.org/tutorials/?tut=ros_control>`_
The :simulation:`config files <march_simulation/config>` contain example controller configuration.

Upload joint names
^^^^^^^^^^^^^^^^^^

The :simulation:`upload_joint_names <march_simulation/scripts/upload_joint_names>` script loads the URDF and uploads the joint names to the topic ``/march/joint_names``.
This is used by the rest of the system to check which joints are active.

Tutorials
---------

Sending commands through the action topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Start the simulation with

.. code::

  roslaunch march_simulation march_world.launch

Then send a command to the controller with

.. code::

  rostopic pub /march/controller/trajectory/command trajectory_msgs/JointTrajectory "
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: ''
  joint_names: [left_hip_aa, left_hip_fe, left_knee, left_ankle, right_hip_aa, right_hip_fe, right_knee,
  right_ankle]
  points:
    -
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: []
      effort: []
      time_from_start:
        secs: 3
        nsecs: 0
        "

You should now see the exoskeleton move in Gazebo.
