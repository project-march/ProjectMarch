import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch file to launch rqt note taker.
    """

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Whether to use simulation time",
            ),
            DeclareLaunchArgument(
                "default_save_directory",
                default_value="note-taker-notes",
                description="Where to autosave the notes, if nothing is done by the "
                "monitor.",
            ),
            Node(
                package="march_rqt_note_taker",
                executable="note_taker",
                output="screen",
                name="note_taker",
                namespace="march",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {
                        "default_save_directory": LaunchConfiguration(
                            "default_save_directory"
                        )
                    },
                ],
            ),
        ]
    )
