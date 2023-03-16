"""Author: Tuhin Das, MVII."""

import getpass
import socket

from rclpy import Future
from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from std_msgs.msg import Header, String
from march_shared_msgs.msg import GaitInstruction, GaitInstructionResponse, CurrentGait, CurrentState, GaitRequest, GaitResponse
from march_shared_msgs.srv import PossibleGaits
from rclpy.node import Node
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH


class WirelessInputDeviceController:
    """The gait controller for the wireless input device.

    Attributes:
        _node (Node): Node that runs the wireless IPD connection manager.
        _logger (Logger): Logger to print information to the terminal.
        _id (str): ID of entity that sends a gait instruction.
        _gait_future (Future): Possible gaits in the current exo state.
    """

    # Format of the identifier for the alive message
    ID_FORMAT = "rqt@{machine}@{user}ros2"

    def __init__(self, node: Node, logger: Logger):
        self._node = node
        self._logger = logger
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())
        self._gait_future = None

        self._send_gait_request = self._node.create_publisher(
            msg_type=GaitRequest,
            topic="/march/gait_request",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._gait_response_subscriber = self._node.create_subscription(
            msg_type=GaitResponse,
            topic="/march/gait_response",
            callback=self._gait_response_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

    def __del__(self):
        """Destroy node cleanly."""
        self._node.destroy_publisher(self._instruction_gait_pub)

    def _gait_response_callback(self, msg: GaitResponse):
        self.get_logger().info("Updating Wireless ipd state...")
        if msg.gait_type is GaitResponse.SIT:
            self._node.self._send_message_till_confirm(msg_type="GaitRequest", requested_gait="sit" )
        if msg.gait_type is GaitResponse.STAND:
            self._node.self._send_message_till_confirm(msg_type="GaitRequest", requested_gait="stand" )
        if msg.gait_type is GaitResponse.WALK:
            self._node.self._send_message_till_confirm(msg_type="GaitRequest", requested_gait="walk" )
        if msg.gait_type is GaitResponse.ERROR:
            self._node.self._send_message_till_confirm(msg_type="fail", requested_gait="" )
        # TODO: Send message to IPD with the newly requested state.

    def get_node(self) -> Node:
        """Simple get function for the node.

        Returns:
            Node: The node object.
        """
        return self._node

    def publish_gait(self, gait_type, walk_type="") -> None:
        """Publish a gait instruction to the gait state machine.

        Args:
            string (str): Name of the gait.
        """
        self._send_gait_request.publish(
            GaitRequest(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                gait_type=gait_type,
                gait_name=walk_type,
                id=str(self._id),
            )
        )
