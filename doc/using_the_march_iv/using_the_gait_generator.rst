.. _using-the-gait-generator-label:

Using the gait generator
========================


.. note:: It is highly recommended to follow the :ref:`robot-model-label` to better understand how ros works with robots.

.. inclusion-introduction-start

This tutorial will teach how to use the |m4| gait generator to create your own gaits and subgaits.

.. inclusion-introduction-end

.. todo:: (Isha) add link to march_gait_generator when it's done.

.. note:: This tutorial covers how to use the gait generator, if you want to develop the gait generator, please check the package description.

The Basics
^^^^^^^^^^
When starting up the gait generator, a number of graphs are opened.
Each graph represents a single joint of the exoskeleton and displays the trajectory for that joint.

.. tip::
  If you want to change the amount of joints you are working with,
  edit the ``<robot>.xacro`` specified in the :gait-generation:`launch file <march_rqt_gait_generator/launch/march_rqt_gait_generator.launch>`

On the y-axes of the graphs the joint angles in degrees are shown. On the x-axes the time in seconds is shown.
The trajectory is defined by several setpoints which are shown as small red circles.
By changing the values of the setpoints, the trajectory can be adjusted.

You can change the values in the table by just clicking on the specific box in the table and adjust the value.
Furthermore, the time and position of a setpoint can be changed by dragging the setpoint within the graph with your mouse.

To add a new setpoint, simply click on the graph.
If you hold ``Ctrl`` while clicking, the new setpoint gets interpolated on the existing trajectory.
To delete a setpoint you have to click on the setpoint that you want to delete while pressing ``Shift``.

Preview
^^^^^^^

Duration
^^^^^^^^

Velocity Markers
^^^^^^^^^^^^^^^^

Mirroring
^^^^^^^^^

Import/Export
^^^^^^^^^^^^^

The structure for storing gait files is explained in :ref:`march-gait-files-label`.
