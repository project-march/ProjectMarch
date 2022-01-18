import getpass
import socket

from rclpy import Future
from std_msgs.msg import Header, String
from rosgraph_msgs.msg import Clock
from march_shared_msgs.msg import Alive, Error, GaitInstruction, GaitInstructionResponse, CurrentGait
from march_shared_msgs.srv import PossibleGaits
from rclpy.node import Node


class InputDeviceController:
    """
    The controller for the input device, uses the node provided in the rqt context.
    Subscriptions:
    - /march/input_device/instruction_response
    - /march/gait/current
    - /clock if the provided node is using simulation time
    Publishers:
    - /march/input_device/instruction
    - /march/error
    - /march/alive if pinging safety node
    """

    # Format of the identifier for the alive message
    ID_FORMAT = "rqt@{machine}@{user}ros2"

    def __init__(self, node):
        self._node = node

        self._instruction_gait_pub = self._node.create_publisher(
            msg_type=GaitInstruction,
            topic="/march/input_device/instruction",
            qos_profile=10,
        )
        self._instruction_response_pub = self._node.create_subscription(
            msg_type=GaitInstructionResponse,
            topic="/march/input_device/instruction_response",
            callback=self._response_callback,
            qos_profile=10,
        )
        self._current_gait = self._node.create_subscription(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            callback=self._current_gait_callback,
            qos_profile=10,
        )
        self._possible_gait_client = self._node.create_client(
            srv_type=PossibleGaits, srv_name="/march/gait_selection/get_possible_gaits"
        )

        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None
        self._possible_gaits = []

        self._id = self.ID_FORMAT.format(
            machine=socket.gethostname(), user=getpass.getuser()
        )

        self.gait_future = None
        self.update_possible_gaits()

    def __del__(self):
        self._node.destroy_publisher(self._instruction_gait_pub)

    def _response_callback(self, msg: GaitInstructionResponse) -> None:
        """
        Callback for instruction response messages.
        Calls registered callbacks when the gait is accepted, finished or rejected.
        The actual callbacks are defined in InputDeviceView

        :type msg: GaitInstructionResponse
        """
        if msg.result == GaitInstructionResponse.GAIT_ACCEPTED and callable(
            self.accepted_cb
        ):
            self.accepted_cb()
        elif msg.result == GaitInstructionResponse.GAIT_FINISHED and callable(
            self.finished_cb
        ):
            self.finished_cb()
        elif msg.result == GaitInstructionResponse.GAIT_REJECTED and callable(
            self.rejected_cb
        ):
            self.rejected_cb()

    def _current_gait_callback(self, msg: CurrentGait) -> None:
        """
        Callback for when the current gait changes, sends the msg through to public current_gait_callback
        :param msg: The string with the name of the current gait
        :type msg: String
        """
        if callable(self.current_gait_cb):
            self.current_gait_cb(msg.gait)

    def update_possible_gaits(self) -> None:
        """
        Send out an asynchronous request to get the possible gaits and stores response in gait_future
        """
        if self._possible_gait_client.service_is_ready():
            self.gait_future = self._possible_gait_client.call_async(
                PossibleGaits.Request()
            )
        else:
            while not self._possible_gait_client.wait_for_service(timeout_sec=1):
                self._node.get_logger().warn("Failed to contact possible gaits service")

    def get_possible_gaits(self) -> Future:
        """
        Returns the future for the names of possible gaits.
        :return: Future for the possible gaits
        """
        return self.gait_future

    def get_node(self) -> Node:
        """
        Simple get function for the node
        :return: the node
        """
        return self._node

    def publish_gait(self, string) -> None:
        self._node.get_logger().debug("Wireless Input Device published gait: " + string)
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.GAIT,
                gait_name=string,
                id=str(self._id),
            )
        )

    def publish_stop(self) -> None:
        self._node.get_logger().debug("Wireless input device published stop")
        msg = GaitInstruction(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            type=GaitInstruction.STOP,
            gait_name="",
            id=str(self._id),
        )
        self._instruction_gait_pub.publish(msg)

    def publish_continue(self) -> None:
        self._node.get_logger().debug("Wireless Input Device published continue")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.CONTINUE,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_pause(self) -> None:
        self._node.get_logger().debug("Wireless Input Device published pause")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.PAUSE,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_sm_to_unknown(self) -> None:
        self._node.get_logger().debug(
            "Wireless Input Device published state machine to unknown"
        )
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.UNKNOWN,
                gait_name="",
                id=str(self._id),
            )
        )
