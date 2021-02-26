"""This module contains some generic functions for python nodes."""

from typing import Optional, List

import rclpy
from march_shared_msgs.srv import GetJointNames
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.client import Client
from urdf_parser_py import urdf

SERVICE_TIMEOUT = 1


def get_robot_urdf(node: Node) -> urdf.Robot:
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


def get_joint_names(node: Node) -> List[str]:
    """Get the joint names from the robot information node."""
    joint_names_client = node.create_client(
        srv_type=GetJointNames,
        srv_name="/march/robot_information/get_joint_names",
    )
    wait_for_service(node, joint_names_client)

    future = joint_names_client.call_async(request=GetJointNames.Request())
    rclpy.spin_until_future_complete(node, future)

    return future.result().joint_names
