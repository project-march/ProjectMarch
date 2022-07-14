"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera_back', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "_109122070820", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud',            'default': 'false', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method',             'default': '2', 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': "ERROR", 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'log', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_width',                  'default': '-1', 'description': 'depth image width'},
                           {'name': 'depth_height',                 'default': '-1', 'description': 'depth image height'},
                           {'name': 'enable_depth',                 'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'color_width',                  'default': '-1', 'description': 'color image width'},
                           {'name': 'color_height',                 'default': '-1', 'description': 'color image height'},
                           {'name': 'enable_color',                 'default': 'false', 'description': 'enable color stream'},
                           {'name': 'infra_width',                  'default': '-1', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '-1', 'description': 'infra width'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width',                'default': '-1', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '-1', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width',             'default': '-1', 'description': 'depth image width'},
                           {'name': 'confidence_height',            'default': '-1', 'description': 'depth image height'},
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps',                  'default': '-1.', 'description': ''},
                           {'name': 'depth_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'confidence_fps',               'default': '-1.', 'description': ''},
                           {'name': 'infra_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'color_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'gyro_fps',                     'default': '400', 'description': ''},
                           {'name': 'accel_fps',                    'default': '250', 'description': ''},
                           {'name': 'color_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'depth_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos',                  'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'infra_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'pointcloud_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro',                  'default': 'true', 'description': ''},
                           {'name': 'enable_accel',                 'default': 'true', 'description': ''},
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': ''},
                           {'name': 'align_depth',                  'default': 'false', 'description': ''},
                           {'name': 'filters',                      'default': "''", 'description': ''},
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},
                           {'name': 'initial_reset',                'default': 'true', 'description': ''},
                           {'name': 'allow_no_texture_points',      'default': 'true', 'description': ''},
                           {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "odom_in", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'temporal.holes_fill',          'default': '0', 'description': 'Persistency mode'},
                           {'name': 'stereo_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    log_level = 'info'
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            ),
            DeclareLaunchArgument(
                name="use_mag",
                default_value="False",
            ),
            DeclareLaunchArgument(
                name="publish_tf",
                default_value="False",
            ),
            DeclareLaunchArgument(
                name="world_frame",
                default_value="nwu",
            ),
        launch_ros.actions.Node(
            package="imu_filter_madgwick",
            namespace=LaunchConfiguration("camera_name"),
            name="imu_filter",
            executable="imu_filter_madgwick_node",
            parameters=[
                {"use_mag": LaunchConfiguration("use_mag")},
                {"publish_tf": LaunchConfiguration("publish_tf")},
                {"world_frame": LaunchConfiguration("world_frame")},
            ],
            remappings=[
                ("imu/data_raw", "imu"),
            ],
        ),
    ])
