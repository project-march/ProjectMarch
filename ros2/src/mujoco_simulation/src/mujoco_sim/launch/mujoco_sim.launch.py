"""Author: MVIII."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch description of the mujoco simulation nodes.

    These nodes are started when the mujoco simulation has be run.
    """
    model_to_load = LaunchConfiguration("model_to_load", default="model_to_load.xml")
    tunings_to_load = LaunchConfiguration("tunings_to_load_path")
    aie_force = LaunchConfiguration("aie_force", default="false")

    gaiting_to_load = LaunchConfiguration("gaiting_to_load", default="air")
    obstacle_to_load = LaunchConfiguration("obstacle_to_load", default="")

    return LaunchDescription(
        [
            Node(
                package="mujoco_sim",
                namespace="",
                executable="mujoco_sim_node",
                name="mujoco_sim",
                parameters=[
                    tunings_to_load,
                    {"model_to_load": model_to_load}, 
                    {"aie_force": aie_force},
                    {"gaiting_to_load": gaiting_to_load},
                    {"obstacle_to_load": obstacle_to_load},
                ],
            ),
            Node(
                package="mujoco_reader",
                namespace="",
                executable="mujoco_reader_node",
                name="mujoco_reader",
                parameters=[{"reader_data_type": 0}],
            ),
            Node(
                package="mujoco_writer",
                executable="mujoco_writer_node",
                name="mujoco_writer",
            ),
        ]
    )
