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

    # Load the exo hardware configuration file
    exo_hardware_config_filepath = os.path.join(
                get_package_share_directory('march_hardware_builder'),
                'robots',
                'hardware_izzy.yaml'
            )

    # Use context manager for file operations
    with open(exo_hardware_config_filepath, 'r') as file:
        exo_hardware_config = yaml.safe_load(file)

    # Extract actuator names
    actuator_names = [list(actuator.keys())[0] for actuator in exo_hardware_config['march9']['joints']]
    actuators_info = dict()
    
    for actuator_idx, actuator in enumerate(actuator_names):
        actuator_info = exo_hardware_config['march9']['joints'][actuator_idx][actuator]['motor_controller']['absoluteEncoder']
        actuators_info[actuator] = {
            'name': actuator,
            'cpr_absolute': 1 << actuator_info['resolution'],
            'direction': actuator_info['direction'],
            'min_position_iu': actuator_info['minPositionIU'],
            'max_position_iu': actuator_info['maxPositionIU'],
            'zero_position_iu': actuator_info['zeroPositionIU'],
            'lower_error_soft_limit': np.rad2deg(actuator_info['lowerErrorSoftLimitMarginRad']),
            'upper_error_soft_limit': np.rad2deg(actuator_info['upperErrorSoftLimitMarginRad']),
        }
    
    min_position_degrees = [
        360 * (actuators_info[actuator]['min_position_iu'] - actuators_info[actuator]['zero_position_iu'])/actuators_info[actuator]['cpr_absolute']
        for actuator, info in actuators_info.items()
    ]
    
    max_position_degrees = [
        360 * (actuators_info[actuator]['max_position_iu'] - actuators_info[actuator]['zero_position_iu'])/actuators_info[actuator]['cpr_absolute']
        for actuator, info in actuators_info.items()
    ]

    for actuator in actuators_info:
        if actuators_info[actuator]['direction'] == -1:
            actuators_info[actuator]['min_position_iu'] = -actuators_info[actuator]['max_position_iu']
            actuators_info[actuator]['max_position_iu'] = -actuators_info[actuator]['min_position_iu']
    
    lower_error_soft_limits = [actuators_info[actuator]['lower_error_soft_limit'] for actuator in actuators_info]
    upper_error_soft_limits = [actuators_info[actuator]['upper_error_soft_limit'] for actuator in actuators_info]

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
                {'actuator_lower_error_soft_limits': lower_error_soft_limits},
                {'actuator_upper_error_soft_limits': upper_error_soft_limits},
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