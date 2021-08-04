.. _march-description-label:

march_description
=================
The ``march_description`` package contains all robot descriptions used by Project March.
The robot descriptions are formatted in the `Unified Robot Description Format (URDF) <https://wiki.ros.org/urdf>`_,
URDF is an xml format and is preferred to be written using `xacro <https://wiki.ros.org/xacro>`_, which is
an xml macro language. This enables us to write shorter and more readable robot descriptions.

Since the ROS ``urdf`` package can only parse the URDF format, we must first process the xacro files.
This is done at build time and is described in the :march:`CMakeLists.txt <march_description/CMakeLists.txt>`.
Both the ``*.urdf`` and ``*.xacro`` files are installed into the ``march_description`` package.
In the launch files these files can be uploaded to the parameter server at ``/robot_description``,
so that they are available for all nodes.

.. code::

  <param name="robot_description" textfile="$(find march_description)/urdf/march4.urdf"/>

The ``*.xacro`` files are also installed since xacro allows for file arguments.
This is used in the :simulation:`march_world.launch <march_simulation/launch/march_world.launch>`.

.. code::

  <param name="robot_description"
         command="$(find xacro)/xacro '$(find march_description)/urdf/$(arg robot).xacro'
                  k_velocity_value_hfe:=60.0 k_velocity_value_kfe:=60.0
                  k_velocity_value_haa:=60.0 k_velocity_value_adpf:=60.0
                  k_position_value_hfe:=50.0 k_position_value_kfe:=50.0
                  k_position_value_haa:=50.0 k_position_value_adpf:=50.0
                  max_effort_rotary:=200.0 max_effort_linear:=40.0
                  ground_gait:=$(arg ground_gait)" />

It uses the ``xacro`` command to process the ``*.xacro`` files and add arguments to change the properties
of the robot in simulation.

Aside from the robot descriptions, the ``march_description`` package also contains ``*.stl`` files,
which contain visualization for the robots, since URDF only describes the properties of a robot.
These ``*.stl`` files have been exported from the robot assembly in Catia.
These visualization files are included in the xacro, which can be used by tools like RViz and Gazebo
to visualize the robot.

See also
^^^^^^^^
* :ref:`robot-model-label`
* :ref:`march-simulation-label`
* :ref:`march-launch-label`
