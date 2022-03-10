std_msgs/Header header

# This contains the position of a point in free space, as well as a preprocessed point
geometry_msgs/Point processed_point
geometry_msgs/Point point
geometry_msgs/Point point_world
geometry_msgs/Point displacement

# ROS frames the points reside in
string processed_point_frame
string point_frame

# These contain points between the current foot position and the found foot position
geometry_msgs/Point[] track_points

# This contains the duration of the gait
float32 duration
