"""Author: Katja Schmal, MVI & Marco Bak, MVIII"""
import getpass
import socket

from std_msgs.msg import Header, Bool
from march_shared_msgs.msg import Alive, Error, GaitInstruction, GaitInstructionResponse, GaitRequest, GaitResponse
from rclpy.node import Node


class InputDeviceController:
    """The controller for the input device, uses the node provided in the rqt context.

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

    def __init__(self, node: Node):
        self._node = node

        self._ping = self._node.get_parameter("ping_safety_node").get_parameter_value().bool_value

        self._eeg_on_off_pub = self._node.create_publisher(
            msg_type=Bool,
            topic="/march/eeg/on_off",
            qos_profile=1,
        )
        self._error_pub = self._node.create_publisher(
            msg_type=Error,
            topic="/march/error",
            qos_profile=10
        )
        self._send_gait_request = self._node.create_publisher(
            msg_type=GaitRequest,
            topic="/march/gait_request",
            qos_profile=10,
        )
        self._gait_response_subscriber = self._node.create_subscription(
            msg_type=GaitResponse,
            topic="/march/gait_response",
            callback=self._gait_response_callback,
            qos_profile=10,
        )
        if self._ping:
            self._alive_pub = self._node.create_publisher(
                Alive,
                "/march/input_device/alive",
                10
            )
            self._alive_timer = self._node.create_timer(
                timer_period_sec=0.1,
                callback=self._timer_callback,
                clock=self._node.get_clock(),
            )

        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        self.eeg = False

        self._current_gait = GaitRequest.FORCE_UNKNOWN

    def __del__(self):
        """Deconstructer, that shuts down the publishers and resets the timers."""
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    def _gait_response_callback(self, msg: GaitResponse):
        self._node.get_logger().info("Callback baby")
        self._current_gait = msg.gait_type

    def _timer_callback(self) -> None:
        """Callback to send out an alive message."""
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)

    def get_node(self) -> Node:
        """Get function for the node.

        Returns:
             Node. the node that runs the input_device_controller.
        """
        return self._node

    def publish_gait(self, gait_type: int) -> None:
        """Publish a message on `/march/input_device/instruction` to publish the gait."""
        self._node.get_logger().info("Mock Input Device published gait: " + str(gait_type))
        self._current_gait = gait_type
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=gait_type,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)

    def publish_stop(self) -> None:
        """Publish a message on `/march/input_device/instruction` to stop the gait."""
        self._node.get_logger().debug("Mock input device published stop")
        self._current_gait = GaitRequest.STAND
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=GaitRequest.STAND,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)

    def publish_error(self) -> None:
        """Publish a fake error message on `/march/error`."""
        self._node.get_logger().debug("Mock Input Device published error")
        self._current_gait = GaitRequest.ERROR
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=GaitRequest.ERROR,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)

    def publish_sm_to_unknown(self) -> None:
        """Publish a message on `/march/input_device/instruction` that has an unknown instruction."""
        self._node.get_logger().debug("Mock Input Device published state machine to unknown")
        self._current_gait = GaitRequest.FORCE_UNKNOWN
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=GaitRequest.FORCE_UNKNOWN,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)

    def publish_eeg_on_off(self) -> None:
        """Publish eeg on if its off and off if it is on."""
        self._eeg_on_off_pub.publish(Bool(data=not self.eeg))

    def update_eeg_on_off(self, msg: Bool) -> None:
        """Update eeg value for when it is changed in the state machine."""
        self.eeg = msg.data

    @property
    def node(self):
        return self._node
