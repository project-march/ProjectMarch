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

    # Extract joint names
    joint_names = [list(joint.keys())[0] for joint in exo_hardware_config['march9']['joints']]
 
    # Calculate joint information and store it in a dictionary
    joints_info = {}
    for joint_idx, joint in enumerate(joint_names):
        joint_info = exo_hardware_config['march9']['joints'][joint_idx][joint]['motor_controller']['absoluteEncoder']
        joints_info[joint] = {
            'name': joint,
            'cpr_absolute': 1 << joint_info['resolution'],
            'direction': joint_info['direction'],
            'min_position_iu': joint_info['minPositionIU'],
            'max_position_iu': joint_info['maxPositionIU'],
            'zero_position_iu': joint_info['zeroPositionIU'],
            'lower_soft_limit': np.rad2deg(joint_info['lowerSoftLimitMarginRad']),
            'upper_soft_limit': np.rad2deg(joint_info['upperSoftLimitMarginRad']),
        }
    
    # Calculate min and max position degrees for each joint
    min_position_degrees = [
        360 * (info['min_position_iu'] - info['zero_position_iu']) / info['cpr_absolute']
        for info in joints_info.values()
    ]
    
    max_position_degrees = [
        360 * (info['max_position_iu'] - info['zero_position_iu']) / info['cpr_absolute']
        for info in joints_info.values()
    ]
    
    # Adjust positions based on direction
    for i, info in enumerate(joints_info.values()):
        if info['direction'] == -1:
            min_position_degrees[i], max_position_degrees[i] = -max_position_degrees[i], -min_position_degrees[i]
    
    # Extract lower and upper error soft limits
    lower_soft_limits = [info['lower_soft_limit'] for info in joints_info.values()]
    upper_soft_limits = [info['upper_soft_limit'] for info in joints_info.values()]
    

    return LaunchDescription([
        Node(
            package='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
            parameters=[
                ik_solver_config,
                {'state_estimator_timer_period': state_estimator_clock_period},
                {'joint_names': joint_names},
                {'joint_min_position_degrees': min_position_degrees},
                {'joint_max_position_degrees': max_position_degrees},
                {'joint_lower_soft_limits': lower_soft_limits},
                {'joint_upper_soft_limits': upper_soft_limits},
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