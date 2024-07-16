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
import numpy as np

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
    exo_hardware_config_filepath = os.path.join(
                get_package_share_directory('march_hardware_builder'),
                'robots',
                'hardware_izzy.yaml'
            )

    # Use context manager for file operations
    with open(exo_hardware_config_filepath, 'r') as file:
        exo_hardware_config = yaml.safe_load(file)

    # Extract actuator names
    actuator_names = [actuator for actuator in exo_hardware_config['march9']['joints']]

    # Simplify actuator names extraction and actuators_info construction
    actuators_info = {
        actuator_name: {
            'name': actuator_name,
            'resolution': actuator_info['motor_controller']['absoluteEncoder']['resolution'],
            'direction': actuator_info['motor_controller']['absoluteEncoder']['direction'],
            'min_position_iu': actuator_info['motor_controller']['absoluteEncoder']['minPositionIU'],
            'max_position_iu': actuator_info['motor_controller']['absoluteEncoder']['maxPositionIU'],
            'zero_position_iu': actuator_info['motor_controller']['absoluteEncoder']['zeroPositionIU'],
            'soft_lower_limit': np.rad2deg(actuator_info['motor_controller']['absoluteEncoder']['lowerSoftLimitMarginRad']),
            'soft_upper_limit': np.rad2deg(actuator_info['motor_controller']['absoluteEncoder']['upperSoftLimitMarginRad']),
        }
        for actuator_name, actuator_info in exo_hardware_config['march9']['joints'].items()
    }

    # Calculate cpr_absolute, direction, min_position_degrees, and max_position_degrees
    cpr_absolute = {actuator: 2 << info['resolution'] for actuator, info in actuators_info.items()}
    direction = {actuator: info['direction'] for actuator, info in actuators_info.items()}
    min_position_degrees = {
        actuator: 360 * (info['min_position_iu'] - info['zero_position_iu']) / cpr_absolute[actuator]
        for actuator, info in actuators_info.items()
    }
    max_position_degrees = {
        actuator: 360 * (info['max_position_iu'] - info['zero_position_iu']) / cpr_absolute[actuator]
        for actuator, info in actuators_info.items()
    }

    # Adjust min and max positions based on direction
    for actuator, dir_value in direction.items():
        if dir_value == -1:
            min_position_degrees[actuator], max_position_degrees[actuator] = -max_position_degrees[actuator], -min_position_degrees[actuator]

    # Extract soft limits
    soft_upper_limits = [info['soft_upper_limit'] for info in actuators_info.values()]
    soft_lower_limits = [info['soft_lower_limit'] for info in actuators_info.values()]


    return LaunchDescription([
        Node(
            package='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
            parameters=[
                ik_solver_config,
                {'state_estimator_timer_period': state_estimator_clock_period},
                {'actuator_names': actuator_names},
                {'actuator_min_position_degrees': min_position_degrees},
                {'actuator_max_position_degrees': max_position_degrees},
                {'actuator_soft_upper_limits': soft_upper_limits},
                {'actuator_soft_lower_limits': soft_lower_limits},
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