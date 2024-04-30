from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Basic launch file to launch the march_gait_planning node."""
    return LaunchDescription(
        [
            Node(
                package="march_footstep_planner",
                executable="footstep_planner_node",
                name="footstep_planner_node", 
            ), 
        ]
    )