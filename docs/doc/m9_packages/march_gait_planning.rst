.. _march_gait_planning-label:

march_gait_planning
===================
**Authors**: Femke Buiks, MIX; Andrew Hutani, MIX

Overview
--------
This package contains all the Gait Planning nodes for the MarchIX exoskeleton. 
For now, there are four different implementations for the Gait Planning node, each with their own launch file: 

* Joint angle based gaits
* Cartesian based gaits 
* Test Setup gaits
* Test individual joints gaits 

The gait planning nodes are responsible for calculating the desired foot positions and joint angles for the exoskeleton.
For this, the Joint angles implementation and Cartesian implementation work in tandem by being Life Cycle Nodes. The Gait Planning Manager will instruct the correct nodes to become awake based on the current_mode (somewhat akin to a service client relation).

Based on the current mode, the active GP node will act accordingly, either sending Joint Angles to the Hardware Interface or sending Cartesian foot positions to the Inverse Kinematics Solver.

ROS API
-------
.. image:: ./images/gait_planning_rqt.png
   :alt: alternative text
   :align: center

Nodes
^^^^^
:code:`gait_planning_angles_node` - Joint Angle Based Gaits 

:code:`gait_planning_node`- Cartesian Based Gaits 

:code:`test_setup_gait_planning_node` - Test Setup Gaits (used only on the Test Setup)

:code:`test_joints_gait_planning_node` - Test Individual Joints Gaits (used for checking all the joint individually)

Subscribed Topics
^^^^^^^^^^^^^^^^^
* | :code:`"state_estimation/state"` `StateEstimation <https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/msg/StateEstimation.msg/>`_
  | Published on a 20ms timer, which triggers the gait planning nodes to publish at the same time intervals. Contains current stance leg, joint states (joint angle configurations) and foot positions. 
* | :code:`"gait_planning_mode"` `ExoMode <https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/msg/ExoMode.msg/>`_
  | Contains the current requested mode (or gait type). 
* | :code:`"mpc_solver/buffer/output"` `PoseArray <https://docs.ros2.org/galactic/api/geometry_msgs/msg/PoseArray.html>`_
  | Contians adjusted foot positions from the Model Predictive Control Solver in the body frame.

  .. warning:: This feature is not fully functional yet.


Published Topics
^^^^^^^^^^^^^^^^
* | :code:`"ik_solver/buffer/input"` `IksFootPositions <https://gitlab.com/project-march/march/-/blob/main/ros2/src/shared/march_shared_msgs/msg/IksFootPositions.msg/>`_
  | Published by the *gait_planning_node* to send desired foot positions in cartesian coordinates to the Inverse Kinematics Solver.
* | :code:`"bezier_visualization"` `PoseArray <https://docs.ros2.org/galactic/api/geometry_msgs/msg/PoseArray.html>`_
  | Published by the *gait_planning_node* to send the calculated swing leg bezier trajectory to a visualization node.

  .. warning:: This feature is not used anymore.
* | :code:`"march_joint_position_controller/commands"` `Float64MultiArray <https://docs.ros2.org/galactic/api/std_msgs/msg/Float64MultiArray.html>`_
  | Published by the *gait_planning_angles_node* to send the desired joint angles straight to the Hardware Interface.
