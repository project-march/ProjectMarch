std_msgs/Header header
# Process noise covariances
float64[] process_linear_acceleration
float64[] process_angular_velocity
float64[] process_foot_position
float64[] process_acceleration_bias
float64[] process_gyroscope_bias
float64[] process_foot_slippage
# Observation noise covariances
float64[] observation_foot_position
float64[] observation_foot_slippage
float64[] observation_joint_position