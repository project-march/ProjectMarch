std_msgs/Header header
float64 step_time
sensor_msgs/JointState joint_state
march_shared_msgs/DynamicalJointState dynamical_joint_state
sensor_msgs/Imu imu
geometry_msgs/Pose[] body_sole_pose
geometry_msgs/Pose[] body_ankle_pose
geometry_msgs/Pose[] body_foot_pose
geometry_msgs/Pose[] world_foot_pose
uint8 current_stance_leg
uint8 next_stance_leg

# Sensor fusion
float64 performance_cost
bool sensor_fusion_valid