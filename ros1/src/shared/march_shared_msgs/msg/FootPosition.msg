std_msgs/Header header

# This contains the position of a point in free space
geometry_msgs/Point point

# These contain points between the current foot position and the found foot position
geometry_msgs/Point[] track_points

# This contains the duration of the gait
float32 duration
