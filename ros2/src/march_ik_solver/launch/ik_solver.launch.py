import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    config = os.path.join(
                get_package_share_directory('march_ik_solver'),
                'config',
                'ik_solver.yaml'
            )
    
    # urdf_file_name = 'hennie_with_koen.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('march_description'),
    #     'urdf',
    #     'march8',
    #     urdf_file_name
    # )
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    # march_ik_solver_path = FindPackageShare('march_ik_solver')
    # default_rviz_config_path = os.path.join(
    #     march_ik_solver_path,
    #     'rviz',
    #     'urdf.rviz'
    # )

    # gui = LaunchConfiguration('gui', default='true')
    # rviz_config_file = LaunchConfiguration('rvizconfig', default=default_rviz_config_path)
    # model = LaunchConfiguration('model', default=urdf)

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'
        # ),
        # DeclareLaunchArgument(
        #     name='gui',
        #     default_value='true',
        #     description='Flag to enable joint_state_publisher_gui'
        # ),
        # DeclareLaunchArgument(
        #     'rvizconfig',
        #     default_value=default_rviz_config_path,
        #     description='Path to the RViz config file to use'
        # ),
        # DeclareLaunchArgument(
        #     'model',
        #     default_value=str(urdf),
        #     description='Absolute path to robot urdf file'
        # ),
        # Node(
        #     condition=UnlessCondition(LaunchConfiguration('gui')),
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        # ),
        # Node(
        #     condition=IfCondition(LaunchConfiguration('gui')),
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        # ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time
        #     }],
        # ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time, 
        #         'robot_description': robot_desc
        #     }],
        #     arguments=[urdf],
        # ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', default_rviz_config_path],
        # ),
        Node(
            package='march_ik_solver',
            # namespace='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
            parameters=[config],
        ),
        # Node(
        #     package='march_ik_solver',
        #     # namespace='march_ik_solver',
        #     executable='ik_solver_buffer_node',
        #     name='ik_solver_buffer',
        #     output='screen',
        #     parameters=[config],
        # ),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        #     launch_arguments={
        #         'urdf_package': 'march_description',
        #         'urdf_path': LaunchConfiguration('model'),
        #         'rviz_config': LaunchConfiguration('rvizconfig'),
        #         'jsp_gui': LaunchConfiguration('gui')}.items()
        # ),
            
    ])