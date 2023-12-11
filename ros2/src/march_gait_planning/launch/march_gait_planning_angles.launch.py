from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Basic launch file to launch the march_gait_planning joint angle trajectory node."""
    return LaunchDescription(
        [
            Node(
                package="march_gait_planning",
                executable="joint_angle_gait_node",
                name="gait_planning_node", 
            ), 
        ]
    )