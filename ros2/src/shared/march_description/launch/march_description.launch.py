#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
)
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = []
    march_state_publisher_node = Node()
    return LaunchDescription(declared_arguments + [march_state_publisher_node])
