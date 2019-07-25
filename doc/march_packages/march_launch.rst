march_launch
============

Overview
--------
The march_launch package is a collection of launch files which are used to start the exoskeleton and validate its workings.

It also contains a plugin called the ``march_rqt_software_check``. This plugins allows everyone



ROS API
-------
As this package contains an rqt plugin which can start arbitrary launch files,
it can spawn any amount of nodes, topics and parameters.


Nodes
^^^^^
*march_rqt_software_check* - The rqt plugin that allows you to start software checks.

Parameters
^^^^^^^^^^
If a specific script is written to calculate the result of a check,
it is convention to upload it to ``/check/<check_name>`` (e.g. ``/check/slave_count``)

Tutorials
---------

Adding a local check
^^^^^^^^^^^^^^^^^^^^
Doing x is very easy, just do it.

Adding a new exoskeleton check
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Doing y is a bit more difficult.


FAQ
---

How do I see which checks exist?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The easiest way is to launch the software check plugin and look at which buttons are available:

.. code::

  roslaunch march_launch march_rqt_software_check

Or take a look at the :march-iv:`CheckRunner source code<march_launch/src/march_launch/CheckRunner.py>`.

How do I z?
^^^^^^^^^^^
z is not available at the moment.