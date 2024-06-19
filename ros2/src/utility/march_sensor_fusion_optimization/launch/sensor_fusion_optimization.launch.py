from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for the Bayesian Optimization node
    """
    return LaunchDescription(
        [
            Node(
                package="march_sensor_fusion_optimization",
                executable="sensor_fusion_optimizer_node",
                name="sensor_fusion_optimizer_node",
            ),
        ]
    )
