
.. _march_hardware_interface-label:

march_hardware_interface
========================

Overview
--------
what

ROS API
-------

Nodes
^^^^^
*ble_ipd_node* - Responsible for doing the thing.

Subscribed Topics
^^^^^^^^^^^^^^^^^

*/march/template/command/other* (template_msgs/TemplateCommand)
Does the other thing.

Published Topics
^^^^^^^^^^^^^^^^
*/march/input_device/alive* (`std_msgs/Time <https://docs.ros.org/melodic/api/std_msgs/html/msg/Time.html>`_)
Publish empty alive messages so :ref:`march-safety-label` does not throw an error.

*/march/template/result* (template_msgs/Boolean)
Tells you if it worked

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
