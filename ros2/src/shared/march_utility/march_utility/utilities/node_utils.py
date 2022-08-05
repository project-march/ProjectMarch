"""This module contains some generic functions for python nodes."""
import os
from typing import Optional, List, Callable

import rclpy
from rcl_interfaces.msg import ParameterValue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from march_shared_msgs.srv import GetJointNames
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.client import Client
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py import urdf

SERVICE_TIMEOUT = 1
DEFAULT_HISTORY_DEPTH = 10


def on_urdf_online(node: Node, func: Callable[[urdf.Robot], None]) -> None:
    """Retrieves and parses the urdf when updated and performs the given function with the urdf.

    Side Effect:
        Adds subscription to the node with var name '_urdf_subscription', listining to the topic `/robot_description"',
        which triggers the given function when new information is published there.

    Example:
        ::

            node = CreateNode()
            def init(robot) -> None:
                node.get_logger().warn("Received urdf")
                node._robot = robot
                node.ready = True
            on_urdf_online(node, init)

    Args:
        node (rclpy.Node): The node from which the subscribtion should be added.
        func: The function that takes the urdf.robot and which is executed when the robot description is published.
    """
    qos_profile = QoSProfile(
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        depth=1,
    )

    def cb(robot_desc: String):
        func(urdf.Robot.from_xml_string(robot_desc.data))

    node._urdf_subscription = node.create_subscription(String, "/robot_description", cb, qos_profile)


def get_robot_urdf_from_service(node: Node) -> urdf.Robot:
    """Get the robot description from the robot state publisher.

    Note:
        Outdated, this method caused problems. Use on_urdf_online instead.

    Args:
        node (Node): The node to use for making the request.
    """
    robot_description_client = node.create_client(
        srv_type=GetParameters,
        srv_name="/march/robot_state_publisher/get_parameters",
    )
    wait_for_service(node, robot_description_client, 2)

    robot_future = robot_description_client.call_async(request=GetParameters.Request(names=["robot_description"]))
    rclpy.spin_until_future_complete(node, robot_future)

    return urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)


def wait_for_service(node: Node, client: Client, timeout: Optional[float] = SERVICE_TIMEOUT):
    """Wait for a service to become available.

    Args:
        node (Node): The node to use for logging.
        client (Client): Client of the service.
        timeout (float, Optional): Timeout in seconds to wait before logging again. Default is `SERVICE_TIMEOUT`.
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
    """Gets a list of all the joint names from the '{robot_name}.urdf' that are not fixed."""
    robot = urdf.Robot.from_xml_file(
        os.path.join(
            get_package_share_directory("march_description"),
            "urdf",
            f"{robot_name}.urdf",
        )
    )
    return get_joint_names_from_robot(robot)


def make_get_parameters_request(own_node: Node, other_node_name: str, names: List[str]) -> List[ParameterValue]:
    """Make a request to a GetParameters service of a node.

    Args:
        own_node (Node): The node to make the request from.
        other_node_name (Node): The node to make the request to.
        names (List[str]): Parameter names to request from the node.

    Returns:
        List[ParameterValue]. The values that are retrieved from the service call.
    """
    srv_name = f"{other_node_name}/get_parameters"
    client = own_node.create_client(GetParameters, srv_name)

    while not client.wait_for_service(timeout_sec=1):
        own_node.get_logger().info(f"Waiting for {srv_name} to become available.")

    future = client.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(own_node, future)

    return future.result().values


def get_joint_names_from_robot(robot: urdf.Robot) -> List[str]:
    """Gets a list of all the joint names from the urdf that are not fixed."""
    joint_names = []
    for joint in robot.joints:
        if joint.type != "fixed":
            joint_names.append(joint.name)
    return sorted(joint_names)
