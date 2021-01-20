import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.client import Client
from urdf_parser_py import urdf

SERVICE_TIMEOUT = 1


def get_robot_urdf(node: Node):
    """
    Initialize the robot description by getting it from the robot state
    publisher.

    :param node Node to use for making the request.
    """
    robot_description_client = node.create_client(
        srv_type=GetParameters,
        srv_name="/march/robot_state_publisher/get_parameters",
    )
    while not robot_description_client.wait_for_service(timeout_sec=2):
        node.get_logger().warn(
            "Robot description is not being published, waiting.."
        )

    robot_future = robot_description_client.call_async(
        request=GetParameters.Request(names=["robot_description"])
    )
    rclpy.spin_until_future_complete(node, robot_future)

    return urdf.Robot.from_xml_string(robot_future.result().values[0].string_value)


def wait_for_service(node: Node, client: Client, timeout=SERVICE_TIMEOUT):
    """
    Wait for a service to become available.

    :param node: Node to use for logging
    :param client: Client of the service.
    :param timeout: Optional timeout to wait before logging again
    """
    while client.wait_for_service(timeout_sec=timeout):
        node.get_logger().info(f'Waiting for {client.srv_name} to become available')