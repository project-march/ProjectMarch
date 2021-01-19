from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "minimum_fake_temperature",
                default_value="10",
                description="Lower bound to generate fake temperatures from",
            ),
            DeclareLaunchArgument(
                "maximum_fake_temperature",
                default_value="30",
                description="Upper bound to generate fake temperatures from",
            ),
            Node(
                package="march_fake_sensor_data",
                executable="march_fake_sensor_data_node",
                name="fake_sensor_data",
                output="screen",
                parameters=[
                    {
                        "minimum_temperature": LaunchConfiguration(
                            "minimum_fake_temperature"
                        )
                    },
                    {
                        "maximum_temperature": LaunchConfiguration(
                            "maximum_fake_temperature"
                        )
                    },
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
