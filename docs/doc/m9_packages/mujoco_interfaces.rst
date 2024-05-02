.. _mujoco_simulation-label:

mujoco_interfaces
=================

Overview
--------
This package contains all the custom message and service definitions for the mujoco simulation.

Nodes
^^^^^
This package contains no nodes.

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