"""Author: Tuhin Das, MVII."""

import getpass
import socket

from rclpy import Future
from std_msgs.msg import Header, String
from march_shared_msgs.msg import GaitInstruction, GaitInstructionResponse, CurrentGait, CurrentState
from march_shared_msgs.srv import PossibleGaits
from rclpy.node import Node
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger


class WirelessInputDeviceController:
    """The gait controller for the wireless input device."""

    # Format of the identifier for the alive message
    ID_FORMAT = "rqt@{machine}@{user}ros2"

    def __init__(self, node: Node, logger: Logger):
        self._node = node
        self._logger = logger

        self._instruction_gait_pub = self._node.create_publisher(
            msg_type=GaitInstruction,
            topic="/march/input_device/instruction",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._instruction_response_pub = self._node.create_subscription(
            msg_type=GaitInstructionResponse,
            topic="/march/input_device/instruction_response",
            callback=self._response_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._current_gait = self._node.create_subscription(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            callback=self._current_gait_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._current_state = self._node.create_subscription(
            msg_type=CurrentState,
            topic="/march/gait_selection/current_state",
            callback=self._current_state_callback,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._possible_gait_client = self._node.create_client(
            srv_type=PossibleGaits, srv_name="/march/gait_selection/get_possible_gaits"
        )
        self._start_side_pub = self._node.create_publisher(
            msg_type=String,
            topic="/march/step_and_hold/start_side",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None
        self.current_state_cb = None
        self._possible_gaits = []

        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        self.gait_future = None
        self.update_possible_gaits()

    def __del__(self):
        """Destroy node cleanly."""
        self._node.destroy_publisher(self._instruction_gait_pub)

    def _response_callback(self, msg: GaitInstructionResponse) -> None:
        """Callback for instruction response messages. Calls registered callbacks when the gait is accepted, finished or rejected.

        Args:
            msg (GaitInstructionResponse): the response to the published gait instruction
        """
        if msg.result == GaitInstructionResponse.GAIT_ACCEPTED and callable(self.accepted_cb):
            self.accepted_cb()
        elif msg.result == GaitInstructionResponse.GAIT_FINISHED and callable(self.finished_cb):
            self.finished_cb()
        elif msg.result == GaitInstructionResponse.GAIT_REJECTED and callable(self.rejected_cb):
            self.rejected_cb()

    def _current_gait_callback(self, msg: CurrentGait) -> None:
        """Callback for when the current gait changes, sends the msg through to public current_gait_callback.

        Args:
            msg (CurrentGait): the current gait of the exoskeleton
        """
        if callable(self.current_gait_cb):
            self.current_gait_cb(msg)

    def _current_state_callback(self, msg: CurrentState) -> None:
        """Callback for when the current state changes, sends the msg through to public current_state_callback.

        Args:
            msg (CurrentState): the current state of the exoskeleton
        """
        if callable(self.current_state_cb):
            self.current_state_cb(msg)

    def update_possible_gaits(self) -> None:
        """Send out an asynchronous request to get the possible gaits and stores response in gait_future."""
        if self._possible_gait_client.service_is_ready():
            self.gait_future = self._possible_gait_client.call_async(PossibleGaits.Request())
        else:
            while not self._possible_gait_client.wait_for_service(timeout_sec=1):
                self._logger.warn("Failed to contact possible gaits service")

    def get_possible_gaits(self) -> Future:
        """Returns the future for the names of possible gaits.

        Returns:
            Future: the future of the available gaits
        """
        return self.gait_future

    def get_node(self) -> Node:
        """Simple get function for the node.

        Returns:
            Node: the node object
        """
        return self._node

    def publish_gait(self, string) -> None:
        """Publish a gait instruction to the gait state machine.

        Args:
            string (str): name of the gait
        """
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.GAIT,
                gait_name=string,
                id=str(self._id),
            )
        )

    def publish_stop(self) -> None:
        """Publish a stop instruction to the gait state machine."""
        msg = GaitInstruction(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            type=GaitInstruction.STOP,
            gait_name="",
            id=str(self._id),
        )
        self._instruction_gait_pub.publish(msg)

    def publish_start_side(self, string: str) -> None:
        """Publish which leg should swing first for step and hold.

        Args:
            string (str): left_swing or right_swing
        """
        self._start_side_pub.publish(String(data=string))
