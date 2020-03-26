.. _march-simulation-label:

march_shared_classes
====================

Overview
--------
The march_shared_classes package contains classes that are useful in different workspaces of the project. It currently
only contains the Gait classes.

The Gait classes
----------------
The Gait classes consist of the classes Gait, Subgait, JointTrajectory, Limits and Setpoint. These classes can be used
to store a gait. Every Gait object contains a list of Subgait objects. Every Subgait contains a list containing one
JointTrajectory object for each joint in the exoskeleton. Every JointTrajectory object contains a Limits object and a
list of Setpoints. A JointTrajectory object can interpolate between its setpoints to obtain the trajectory the joint
will follow.

Usage
^^^^^

The Gait classes are typically constructed by reading a gait file. This can be done with

.. code::

  Subgait.from_file(robot, filename)

to read a single .subgait file or

.. code::

  Gait.from_file(robot, filename)

to read all the .subgait files in a gait folder at once.

Communication with the controller usually goes via ros messages. Transform a Subgait to a subgait message through

.. code::

  Subgait.to_subgait_msg()
