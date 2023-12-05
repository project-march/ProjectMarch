"Authors: Femke Buiks and Andrew Hutani, MIX"

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_descripion():
    """Basic launch file to launch the march_gait_planning node."""
    return LaunchDescription(
        [
            Node(
                package="march_gait_planning",
                executable="march_gait_planning",
                name="gait_planning_node", 
            ), 
        ]
    )