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
time limit and/or the maximum number of iterations are reached. This time limit is determined by the frequency of the state estimation. The maximum number of iterations is set to 100k.
The order of the tasks is determined depending on the current exo mode. Furthermore, the solver is capable of handling the constraints of  the exoskeleton such as the joint limits, and the joint velocity limits.

The solver is implemented in C++, and uses the Eigen library for matrix operations, and Pinocchio for fast multibody dynamics computations such as the Jacobian and the forward kinematics.

ROS API
-------

Nodes
^^^^^
*ik_solver_node* - Responsible for solving the inverse kinematics of the exoskeleton.
*ik_manager_node* - Responsible for managing the priority of the different tasks in the inverse kinematics solver depending on the exo mode.

Subscribed Topics
^^^^^^^^^^^^^^^^^

**ik_solver_node**

*/state_estimation/state* (march_shared_msgs/StateEstimation)
The current estimated state of the exoskeleton consisting of the joint angles, joint velocities, and the position and orientation of the feet, etc.
The solver uses current joint positions to calculate the necessary joint positions to reach the desired foot positions.

*/ik_solver/buffer/input* (march_shared_msgs/IksFootPositions)
The desired foot positions in 3D Cartesian space both for the left and right foot in the base frame of the exoskeleton at some timestamp.

*/ik_solver/command* (march_shared_msgs/IksCommand)
Contains the list of tasks to be executed by the solver, given the current exo mode, from least important to most important.

**ik_manager_node**

*/current_mode* (march_shared_msgs/ExoMode)
The current exo mode of the exoskeleton such as *standing*, *large walk*, *small walk*, etc.
This is used to determine which tasks to execute and in which order in the solver.

Published Topics
^^^^^^^^^^^^^^^^

**ik_solver_node**

*/march_joint_position_controller/command* (std_msgs/Float64MultiArray <http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32MultiArray.html>`_)
The desired joint angles of the exoskeleton to reach the desired foot positions. The joint angles are published in alphabetical order of the joint names.

*/ik_solver/status* (march_shared_msgs/IksStatus)
The status of the solver containing the error of each task, the number of iterations, and the time taken to solve the inverse kinematics.

**ik_manager_node**

*/ik_solver/command* (march_shared_msgs/IksCommand)
The current exo mode of the exoskeleton such as *standing*, *large walk*, *small walk*, etc.
This is used to determine which tasks to execute and in which order in the solver.

Parameters
^^^^^^^^^^

**ik_solver_node**

*state_estimator_time_offset* (*double*, default: 0.05)
The timestep between two consecutive state estimation messages in seconds.

*convergence_threshold* (*double[]*, default: [])
The error threshold for each task to consider the task converged. The unit of the error is meters for the motion task, radians for the posture task, and meters for the stability task.

*max_iterations* (*int*, default: 1000)
The maximum number of iterations to solve the inverse kinematics.

*integral_dt* (*double*, default: 0.01)
The timestep to integrate the computed joint velocities to get the joint angles.

*joint.names* (*string[]*, default: [])
The names of the joints of the exoskeleton in kinematic order.

*joint.active* (*bool[]*, default: [])
Flag to indicate whether the joint is controllable or not such that the solver returns the list of joint angles for the controllable joints only. Same order as *joint.names*.

*joint.limits.position.upper* (*double[]*, default: [])
The upper joint limits of the exoskeleton in degrees. Same order as *joint.names*.

*joint.limits.position.lower* (*double[]*, default: [])
The lower joint limits of the exoskeleton in degrees. Same order as *joint.names*.

*joint.limits.position.soft* (*double[]*, default: [])
The margin from the joint limits to consider the joint at the limit. Same order as *joint.names*.

*joint.limits.velocity* (*double[]*, default: [])
The joint velocity limits of the exoskeleton in degrees per second. Same order as *joint.names*.

*task.names* (*string[]*, default: [])
The names of the tasks in the solver in order of priority.

*task.motion.kp* (*double[]*, default: [])
The proportional gains of the motion task in this order: x_left, y_left, z_left, x_right, y_right, z_right.

*task.motion.kd* (*double[]*, default: [])
The derivative gains of the motion task in this order: x_left, y_left, z_left, x_right, y_right, z_right.

*task.motion.ki* (*double[]*, default: [])
The integral gains of the motion task in this order: x_left, y_left, z_left, x_right, y_right, z_right.

*task.posture.kp* (*double[]*, default: [])
The proportional gains of the posture task in this order: pitch_left, pitch_right.

*task.posture.kd* (*double[]*, default: [])
The derivative gains of the posture task in this order: pitch_left, pitch_right.

*task.posture.ki* (*double[]*, default: [])
The integral gains of the posture task in this order: pitch_left, pitch_right.

*task.stability.kp* (*double[]*, default: [])
The proportional gains of the stability task in this order: x, y.

*task.stability.kd* (*double[]*, default: [])
The derivative gains of the stability task in this order: x, y.

*task.stability.ki* (*double[]*, default: [])
The integral gains of the stability task in this order: x, y.

**ik_manager_node**

*exo_modes* (*string[]*, default: [])
The list of all possible exo modes of the exoskeleton.

*stack_of_tasks.[name of exo mode]* (*string[]*, default: [])
The list of tasks to be executed by the solver, given the current exo mode, from least important to most important.