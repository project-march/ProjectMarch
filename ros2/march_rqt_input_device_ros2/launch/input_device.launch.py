import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch.actions.DeclareLaunchArgument(
            'ping_safety_node',
            default_value='true',
            description='Whether the safety node should be pinged'),
        launch_ros.actions.Node(
            package='march_rqt_input_device_ros2', executable='march_rqt_input_device_ros2', output='screen',
            name='input_device', use_sim_time='true', ping_safety_node=launch.substitutions.LaunchConfiguration('node_prefix')),
    ])
