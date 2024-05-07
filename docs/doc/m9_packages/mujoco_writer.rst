.. _mujoco_simulation-label:

mujoco_simulation
=================

Overview
--------
what

ROS API
-------

Nodes
^^^^^

*mujoco_writer* - Responsible for doing the thing.

.. figure:: images/mujoco_writer.svg
   :align: center
   :scale: 100%
   :alt: mujoco_writer

Subscribed Topics
^^^^^^^^^^^^^^^^^

*/march_joint_position_controller/commands* (std_msgs/Float64MultiArray)

*/mujoco_reset_trajectory* (std_msgs/Bool)

Published Topics
^^^^^^^^^^^^^^^^
*/mujoco_input* (mujoco_interfaces/MujocoInput)
Services
^^^^^^^^
*/march/template/do* (template_msgs/Do)
Does something

Parameters
^^^^^^^^^^
*/march/template/counter* (*int*, required)
How many to count
*/march/template/countings* (*int[]*, default: [])
List of countings

Tutorials
---------

How to do something
^^^^^^^^^^^^^^^^^^^ 
explain how to do something, for example:

Create a new publisher
^^^^^^^^^^^^^^^^^^^^^^
Create a new publisher in the ``__init__`` of ``InputDeviceController``:

.. code::

from std_msgs.msg import Bool # Import the Bool msg if needed.

self.this_tutorial_works_pub = rospy.Publisher('/march/this/tutorial/works', Bool, queue_size=10)