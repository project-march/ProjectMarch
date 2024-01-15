from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
      LifecycleNode(package='march_lifecycle_test', executable='lifecycle_talker',
                    name='lc_talker', namespace='', output='screen'),
      Node(package='march_lifecycle_test', executable='listener', output='screen'),
      Node(package='march_lifecycle_test', executable='service_client', output='screen')
  ])