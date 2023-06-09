"""Author: Katja Schmal, MVI & Marco Bak, MVIII."""
import getpass
import socket

from std_msgs.msg import Header, Bool, Int32
from march_shared_msgs.msg import Alive, Error, GaitRequest, GaitResponse
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

    # Valid transition that can be made from the current state
    POSSIBLE_TRANSITIONS = {
        GaitRequest.SIT: ["stand"],
        GaitRequest.STAND: ["sit", "walk", "step_and_close", "stop"],
        GaitRequest.WALK: ["stop"],
        GaitRequest.STEP_CLOSE: ["walk", "stand", "stop"],
        GaitRequest.FORCE_UNKNOWN: ["home_stand", "home_sit", "stop"],
        GaitRequest.ERROR: []
    }

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
            topic="/march/input_device/error",
            qos_profile=10
        )
        self._send_gait_request = self._node.create_publisher(
            msg_type=GaitRequest,
            topic="/march/gait_request",
            qos_profile=10,
        )
        self._swing_leg_command_pub = self._node.create_publisher(
            msg_type=Int32,
            topic="/publish_swing_leg_command",
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

    def update_possible_gaits(self):
        """Update the possible gait that can be selected by the IPD."""
        return self.POSSIBLE_TRANSITIONS[self._current_gait]

    def _gait_response_callback(self, msg: GaitResponse):
        """Update current node from the state machine."""
        self._node.get_logger().debug("Received new gait from other IPD.")
        self._current_gait = msg.gait_type

    def publish_gait(self, gait_type: int) -> None:
        """Publish a message on `/march/gait_request` to publish the gait."""
        self._node.get_logger().debug("Mock Input Device published gait: " + str(gait_type))
        self._current_gait = gait_type
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=gait_type,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)
        if gait_type == 5:
            error_msg = Error()
            error_msg.error_message = "Error button clicked on IPD"
            error_msg.type = 0
            self._error_pub.publish(error_msg)

    def publish_eeg_on_off(self) -> None:
        """Publish eeg on if its off and off if it is on."""
        self._eeg_on_off_pub.publish(Bool(data=not self.eeg))

    def update_eeg_on_off(self, msg: Bool) -> None:
        """Update eeg value for when it is changed in the state machine."""
        self.eeg = msg.data

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

    def __del__(self):
        """Deletion function, that shuts down the publishers and resets the timers."""
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    @property
    def node(self):
        """Define the node."""
        return self._node
