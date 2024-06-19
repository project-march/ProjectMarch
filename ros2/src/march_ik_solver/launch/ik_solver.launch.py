import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    state_estimator_clock_period = LaunchConfiguration('state_estimator_timer_period', default='0.05')
    test = LaunchConfiguration('test', default='false')

    ik_solver_config = os.path.join(
                get_package_share_directory('march_ik_solver'),
                'config',
                'ik_solver.yaml'
            )
    
    ik_manager_config = os.path.join(
                get_package_share_directory('march_ik_solver'),
                'config',
                'ik_manager.yaml'
            )
    
    ik_test_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_ik_test"),
                "launch",
                "ik_test.launch.py",
            ),
        ),
        condition=IfCondition(test),
    )

    # Load the soft joint limits from the robot config
    joints_airgaiting_config_filepath = os.path.join(
                get_package_share_directory('march_hardware_builder'),
                'robots', 'Izzy',
                'joints_airgaiting.yaml'
            )
    joints_airgaiting_config = yaml.safe_load(open(joints_airgaiting_config_filepath, 'r'))
    actuator_names = [list(actuator.keys())[0] for actuator in joints_airgaiting_config['march9']['joints']]
    print(actuator_names)

    return LaunchDescription([
        Node(
            package='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
            parameters=[
                ik_solver_config,
                {'state_estimator_timer_period': state_estimator_clock_period},
            ],
        ),
        Node(
            package='march_ik_solver',
            executable='ik_manager_node',
            name='ik_manager',
            output='screen',
            parameters=[ik_manager_config],
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
            arguments=['--layout', os.path.join(get_package_share_directory('march_ik_solver'), 
                                                'plotjuggler', 'ik_solver_multiplot.xml')],
            condition=IfCondition(test),
        ),
        ik_test_node,
    ])