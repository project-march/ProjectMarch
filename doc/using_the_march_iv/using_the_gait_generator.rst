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
On the right side of the screen, a visualization of the exoskeleton is shown.
A preview of the gait that you are generating in the gait generator is shown here.
You can play the preview by clicking on the ``start`` button and stop the preview by clicking on the ``stop`` button.
By adjusting the value of 100, you can change the speed at which the preview is shown.
The speed is defined as a percentage of the actual speed.

Duration
^^^^^^^^
In the bottom right of the gait generator it is possible to change the duration of the gait file.
By ticking the box ``Scale Setpoints``, you can scale all the setpoints to the new duration of the gait, so that the relative amount of time between the setpoints will stay the same.
Without ticking this box the total amount of time of the gait will change without changing the absolute amount of time between setpoints.
This means, when decreasing the total amount of time it is possible that some of the last setpoints will be deleted as they exceed the new duration.
When this happens, a warning will be given by the gait generator.

Velocity Markers
^^^^^^^^^^^^^^^^
When ticking on the box with ``Velocity Markers``, the velocity markers are shown as green lines for each setpoint.
The lines represent the velocity for each setpoint in degrees per second.

Mirroring
^^^^^^^^^
The feature called mirroring allows you to mirror a gait. This means, for the mirrored gait the trajectories of the left and right joints are switched.
One requirement to do this is that the last setpoint of the left leg has to have the same values as the first setpoint of the right leg and the other way around.
Otherwise the gait generator will give you a warning that gait mirroring is not possible. Furthermore, either the word ‘left’ or ‘right’ should be in the name of the subgait.
The mirrored version will be saved with the same version name as the original file, but the word ‘left’ or ‘right’ in the subgait folder is changed to the opposite one.

.. tip::

  Mirroring is not exclusive to gaits and joints with ``left`` or ``right``, you can set custom keys in the settings menu bottom right.

Import/Export
^^^^^^^^^^^^^
The structure for storing gait files is explained in :ref:`march-gait-files-label`.

To export a subgait file, choose a directory where you want to store your subgait.
The directory you choose has to be a level higher than the gait folder.
So choose for example 'airgait' as a directory.
You can save the gait files by clicking on the button ``export`` in the gait generator.
A file can be imported by pressing the ``import`` button and selecting the file.

Publish
^^^^^^^
The ``publish`` button allows you to directly publish the subgait you made on a custom topic.
