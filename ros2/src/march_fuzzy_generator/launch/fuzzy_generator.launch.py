from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate the launch description for the fuzzy generatornode."""
    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")
    ld.add_action(arg)

    default_config = os.path.join(get_package_share_directory("march_fuzzy_generator"), "config", "default_weights.yaml")

    # parameters
    config_path = LaunchConfiguration("config_path", default=default_config)

    fuzzy_generator_node = Node(
        package="march_fuzzy_generator", executable="fuzzy_generator_node", name="fuzzy_generator", parameters=[{"config_path": config_path}]
    )
    ld.add_action(fuzzy_generator_node)

    return ld
