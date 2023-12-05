"""Authors: Femke Buiks and Andrew Hutani, MIX"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Basic launch file to launch the march_gait_planning node."""
    return LaunchDescription(
        [
            Node(
                package="march_gait_planning",
                executable="gait_planning_node",
                name="gait_planning_node", 
            ), 
        ]
    )