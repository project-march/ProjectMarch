.. _march_ik_solver-label:

march_ik_solver
===============

Overview
--------
This package contains the inverse kinematics solver for the exoskeleton. It is responsible for calculating the joint angles of the exoskeleton based on the desired position of the end effector, 
in this case, the position of each foot in 3D Cartesian space. The solver is based on task priority inverse kinematics, where the tasks are the position of each foot known as *motion*, the orientation 
of each foot called *posture*, and the position of the center of gravity of the exoskeleton (including the pilot) called *stability*. The tasks are prioritized in the following order: *stability*, 
*motion*, and *posture*, where the stability task is the most important, and the posture task is the least important. The solver is based on the following equation:

..math::
    \begin{align*}
    \mathbf{\dot{q}}_{desired} = \mathbf{J}_{stability}^{\dagger} \mathbf{e}_{stability} + (\mathbf{I} - \mathbf{J}_{stability}^{\dagger} \mathbf{J}_{stability}) \mathbf{J}_{motion}^{\dagger} \mathbf{e}_{motion} + (\mathbf{I} - \mathbf{J}_{motion}^{\dagger} \mathbf{J}_{motion}) \mathbf{J}_{posture}^{\dagger} \mathbf{e}_{posture}
    \mathbf{q}_{desired} = \mathbf{q}_{current} + \sum_{i=1}^{N} \mathbf{\dot{q}}_{desired,i} \Delta t
    \end{align*}

The solver iteratively calculates the necessary joint angles to reach the desired foot positions starting from the current joint angles, until the error of each task is below a certain threshold, or the 
time limit and/or the maximum number of iterations are reached. This time limit is determined by the frequency of the state estimation. The maximum number of iterations is set to 100.000.
The order of the tasks is determined depending on the current exo mode. Furthermore, the solver is capable of handling the constraints of  the exoskeleton such as the joint limits, and the joint velocity limits.

The solver is implemented in C++, and uses the Eigen library for matrix operations, and Pinocchio for the multibody dynamics computations such as the Jacobian and the forward kinematics.

ROS API
-------

Nodes
^^^^^
*ik_solver_node* - Responsible for solving the inverse kinematics of the exoskeleton.
*ik_manager_node* - Responsible for managing the priority of the different tasks in the inverse kinematics solver depending on the exo mode.

Subscribed Topics
^^^^^^^^^^^^^^^^^

*/state_estimation/state* (march_shared_msgs/StateEstimation)
The current estimated state of the exoskeleton consisting of the joint angles, joint velocities, and the position and orientation of the feet, etc.
The solver uses current joint positions to calculate the necessary joint positions to reach the desired foot positions.

*/ik_solver/buffer/input* (march_shared_msgs/IksFootPositions)
The desired foot positions in 3D Cartesian space both for the left and right foot in the base frame of the exoskeleton at some timestamp.

*/ik_solver/command* (march_shared_msgs/IksCommand)
Contains the list of tasks to be executed by the solver, given the current exo mode, from least important to most important.

Published Topics
^^^^^^^^^^^^^^^^
*/march/input_device/alive* (`std_msgs/Time <https://docs.ros.org/melodic/api/std_msgs/html/msg/Time.html>`_)
Publish empty alive messages so :ref:`march-safety-label` does not throw an error.

*/march/template/result* (template_msgs/Boolean)
Tells you if it worked

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
