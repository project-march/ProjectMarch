"""This module contains some generic functions for python nodes."""
import os
from typing import Optional, List

import rclpy
from march_shared_msgs.srv import GetJointNames
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.client import Client
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py import urdf

SERVICE_TIMEOUT = 1


def get_robot_urdf_from_service(node: Node) -> urdf.Robot:
    """Get the robot description from the robot state publisher.

    :param node Node to use for making the request.
    """
    robot_description_client = node.create_client(
        srv_type=GetParameters,
        srv_name="/march/robot_state_publisher/get_parameters",
    )
    wait_for_service(node, robot_description_client, 2)

    robot_future = robot_description_client.call_async(
        request=GetParameters.Request(names=["robot_description"])
    )
    rclpy.spin_until_future_complete(node, robot_future)

    return urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)


def wait_for_service(
    node: Node, client: Client, timeout: Optional[float] = SERVICE_TIMEOUT
):
    """
    Wait for a service to become available.

    :param node: Node to use for logging
    :param client: Client of the service.
    :param timeout: Optional timeout to wait before logging again
    """
    while not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().info(f"Waiting for {client.srv_name} to become available")


def get_joint_names_from_service(node: Node) -> List[str]:
    """Get the joint names from the robot information node."""
    joint_names_client = node.create_client(
        srv_type=GetJointNames,
        srv_name="/march/robot_information/get_joint_names",
    )
    wait_for_service(node, joint_names_client)

    future = joint_names_client.call_async(request=GetJointNames.Request())
    rclpy.spin_until_future_complete(node, future)

    return future.result().joint_names


def get_joint_names_from_robot_name(robot_name: str) -> List[str]:
    robot = urdf.Robot.from_xml_file(
        os.path.join(
            get_package_share_directory("march_description"),
            "urdf",
            f"{robot_name}.urdf",
        )
    )
    return get_joint_names_from_robot(robot)


def get_joint_names_from_robot(robot: urdf.Robot) -> List[str]:
    joint_names = []
    for joint in robot.joints:
        if joint.type != "fixed":
            joint_names.append(joint.name)
    return sorted(joint_names)
