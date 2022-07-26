"""Atuhor: George Vegelien, MVII."""
import launch.conditions
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    obstacle = LaunchConfiguration("obstacle")
    gazebo_ui = LaunchConfiguration("gazebo_ui")
    debug = LaunchConfiguration("gazebo_debug")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fixed = LaunchConfiguration("fixed")
    verbose = LaunchConfiguration("gazebo_verbose")
    robot = LaunchConfiguration("robot")

    # region Declared arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "obstacle",
            default_value="none",
            description="Obstacle to load in the simulation.",
            choices=["none", "bench", "curb", "ramp", "rough_terrain", "stairs", "tilted_path"]
        ),
        DeclareLaunchArgument(
            "gazebo_ui",
            default_value="true",
            description="Launches the Gazebo UI.",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "gazebo_debug",
            default_value="false",
            description="Starts gazebo debugging with gdb.",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Uses simulated time and publishes on /clock.",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name='fixed',
            default_value='true',
            description='Fixes the exoskeleton in the world. Transforms baselink to world if not fixed.',
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name='gazebo_verbose',
            default_value='true',
            description='If gazebo should print errors and logs.',
            choices=["true", "false"]
        ),
    ]
    # endregion

    # region Launch Gazebo

    gazebo_world_file = PathJoinSubstitution([
        get_package_share_directory('march_simulation'),
        'worlds', 'march.world'
    ])

    # The idea is that this should stop previously open gazebo processes. This is NOT EXTENSIVELY TESTED.
    close_gazebo = ExecuteProcess(
        cmd=[['killall', '-9', 'gzserver', 'gzclient']],
        shell=True
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments=[
            ("world_name", gazebo_world_file),
            ("paused", "false"),
            ("use_sim_time", use_sim_time),  # default 'True'.
            ("gui", gazebo_ui),  # default 'True'
            ("recording", "false"),  # Enable gazebo state log recording
            ("debug", debug),  # default 'false'
            ("verbose", verbose),  # default 'true'
            ("server_required", "false"),  # If set to 'True' crashes ros on startup, if 'False' entities persist.
        ],
    )

    # Will create an entity with the name -entity [name] based on the urdf in the topic -topic [topic]
    # Check this link for more information: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete
    gazebo_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot],
        output="screen",
    )
    # endregion

    # region Spawn obstacle
    spawn_obstacle = Node(
        package='march_simulation',
        executable='spawn_obstacle',
        parameters=[{'obstacle': obstacle}],
        output='screen',
        condition=launch.conditions.LaunchConfigurationNotEquals("obstacle", "none")
    )
    # endregion

    # region Spawn obstacle dimension setter
    obstacle_dimension_setter = Node(
        name="march_set_obstacle_dimension_node",
        package="march_simulation",
        executable="set_obstacle_dimensions"
    )
    # endregion

    # region Spawn node that transforms baselink to world
    base_link_to_world_transform = Node(
        package='march_simulation',
        executable='to_world_transform',
        name='world_transformer',
        output='screen',
        condition=UnlessCondition(fixed)
    )
    # endregion

    nodes = [
        close_gazebo,
        gazebo,
        gazebo_spawn_entity,
        spawn_obstacle,
        obstacle_dimension_setter,
        base_link_to_world_transform
    ]

    return LaunchDescription(declared_arguments + nodes)
