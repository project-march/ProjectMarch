import launch
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
            package='march_rqt_input_device_ros2', executable='input_device', output='screen',
            name='input_device_launch', parameters=[{'use_sim_time': True}])
    ])
