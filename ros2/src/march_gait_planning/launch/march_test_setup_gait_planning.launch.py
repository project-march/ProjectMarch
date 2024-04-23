from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Basic launch file to launch the march_gait_planning node."""
    return LaunchDescription(
        [
            Node(
                package="march_gait_planning",
                executable="test_setup_gait_planning_node",
                name="test_setup_gait_planning_node", 
            ), 
        ]
    )