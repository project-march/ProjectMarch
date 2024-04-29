.. _march_gait_planning-label:

march_gait_planning
===================

Overview
--------
This package contains all the Gait Planning nodes for the March9 exo. For now, there are four different implementations for the Gait Planning node, each with their own launch file: 
* Joint angle based gaits
* Cartesian based gaits 
* Test Setup gaits
* Test individual joints gaits 

ROS API
-------

Nodes
^^^^^
*gait_planning_angles_node* - joint angle based gaits 
*gait_planninbg_node* - cartesian based gaits 
*test_setup_gait_planning_node* - Test Setup gaits
*test_joints_gait_planning_node* - individual joint gaits 

Subscribed Topics
^^^^^^^^^^^^^^^^^
*state_estimation/state*
Published on a 20ms timer, which triggers the gait planning nodes to publish at the same time intervals. Contains current stance leg, joint states (joint angle configurations) and foot positions. 

*current_mode*
Contains the current requested mode (or gait type) from the Input Device. 


Published Topics
^^^^^^^^^^^^^^^^
*ik_solver/buffer/input* 
Published by the *gait_planning_node* to send desired foot positions in cartesian coordinates to the Inverse Kinematics Solver. 

*bezier_visualization* 
Published by the *gait_planning_node* to send the calculated swing leg bezier trajectory to a visualization node. 

*march_joint_position_controller/commands* 
Published by the *gait_planning_angles_node* to send the desired joint angles straight to the Hardware Interface. 

Services/Clients
^^^^^^^^^^^^^^^^


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
