.. _march_state_estimator-label:

march_state_estimator
=====================

Overview
--------
This package contains the state estimator of the exoskeleton. The state estimator is responsible for estimating the internal state of the exoskeleton based on the sensor data.
In addition, it also acts as a synchronization between all nodes such that they can work with the same timestamp with a rate of 200 Hz.


The state estimator is based on the Extended Kalman Filter (EKF) algorithm designed by `Rotella et al. (2014) <https://arxiv.org/abs/1402.5450>`_, which is a recursive Bayesian filter 
that estimates the state of a dynamic system from a series of noisy measurements. The state estimator is implemented in C++, and uses the Eigen library for matrix operations, and the ROS library for communication with other nodes.

The state estimator estimates the following internal states of the exoskeleton:
- Joint states
- Dynamical joint states
- IMU data
- Sole poses in body frame
- Ankle poses in body frame
- Foot poses in body frame
- Foot poses in world frame
- Current stance leg
- Next stance leg

ROS API
-------

Nodes
^^^^^
*state_estimator_node* - Responsible for estimating the internal state of the exoskeleton based on the sensor data.
*torque_converter_node* - Responsible for converting the desired joint torques to the motor torques based on the desired joint angles and velocities.
*filter_node* - Responsible for smoothening the sensor data, such as the joint states and the IMU data.

Subscribed Topics
^^^^^^^^^^^^^^^^^

*/joint_states/filtered* (sensor_msgs/JointState)
The joint states of the exoskeleton consisting of the joint angles, joint velocities, and joint efforts.

*/lower_imu/filtered* (sensor_msgs/Imu)
The IMU data of the exoskeleton mounted on the backpack.

*/lower_imu/position* (geometry_msgs/PointStamped)
The position of the IMU on the backpack of the exoskeleton in the world frame (only in simulation).

*/lower_imu/velocity* (geometry_msgs/Vector3Stamped)
The velocity of the IMU on the backpack of the exoskeleton in the world frame (only in simulation).

Published Topics
^^^^^^^^^^^^^^^^
*/march/input_device/alive* (`std_msgs/Time <https://docs.ros.org/melodic/api/std_msgs/html/msg/Time.html>`_)
Publish empty alive messages so :ref:`march-safety-label` does not throw an error.

*/march/template/result* (template_msgs/Boolean)
Tells you if it worked

*/state_estimation/clock* (std_msgs/Header)
The current timestamp of the state estimator. This is used to synchronize the nodes with the same timestamp.

*/state_estimation/state* (march_shared_msgs/StateEstimation)
The estimated internal state of the exoskeleton consisting of the joint states, dynamical joint states, IMU data, sole poses, ankle poses, foot poses, world foot poses, current stance leg, and next stance leg.

*/state_estimation/ground_reaction_force/left* (geometry_msgs/Vector3Stamped)
The ground reaction force of the left foot in the left foot frame. This is for debugging purposes.

*/state_estimation/ground_reaction_force/right* (geometry_msgs/Vector3Stamped)
The ground reaction force of the right foot in the right foot frame. This is for debugging purposes.

*/est_foot_positions* (geometry_msgs/PoseArray)
The estimated foot positions in the right foot frame. This is used for ZMP-based MPC (deprecated).

*/robot_com_position* (march_shared_msgs/CenterOfMass)
The estimated center of mass position and velocity of the exoskeleton in the right foot frame. This is used for ZMP-based MPC (deprecated).

*/robot_zmp_position* (geometry_msgs/PointStamped)
The estimated zero moment point (ZMP) position of the exoskeleton in the right foot frame. This is used for ZMP-based MPC (deprecated).

*/current_stance_foot* (std_msgs/Int32)
The current stance foot of the exoskeleton (-1 for left, 0 for double, 1 for right). This is used for ZMP-based MPC (deprecated).

*/state_estimation/com_position* (geometry_msgs/PointStamped)
The estimated center of mass position of the exoskeleton in the right foot frame. This is for visualization purposes.

Parameters
^^^^^^^^^^
*/robot_definition* (*string*, required)
The name of the robot definition file of the exoskeleton described as set of equations.

*/urdf_file_path* (*string*, required)
The path to the URDF file of the exoskeleton.

*/clock_period* (*double*, default: 0.005)
The period of the clock in seconds.

*/left_stance_threshold* (*double*, default: 400.0)
The threshold of the ground reaction force of the left foot to determine the stance leg in Newton.

*/right_stance_threshold* (*double*, default: 400.0)
The threshold of the ground reaction force of the right foot to determine the stance leg in Newton.
