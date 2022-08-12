"""Author: Jelmer de Wolde, MVII."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

default_buffer_size = 15
layout = "positions_and_effort.xml"

package_directory = get_package_share_directory("march_plotjuggler_launcher")
layout_location = package_directory + "/config/" + layout


def generate_launch_description():
    """Launchfile to launch plotjuggler with the given buffer size and layout."""
    print(layout_location)
    return LaunchDescription(
        [
            Node(
                package="plotjuggler",
                namespace="plotjuggler",
                executable="plotjuggler",
                name="plotjuggler",
                arguments=[
                    "-n",
                    "--buffer_size",
                    str(default_buffer_size),
                    "-l",
                    layout_location,
                ],
            ),
        ]
    )
