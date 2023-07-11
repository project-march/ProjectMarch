"""Author: Katja Schmal, MVI & Marco Bak, MVIII."""
import getpass
import socket

from march_shared_msgs.msg import GaitRequest, GaitResponse
from rclpy import Future
from std_msgs.msg import Header, String, Bool, Int32
from rosgraph_msgs.msg import Clock
from march_shared_msgs.msg import Alive, Error, GaitInstruction, GaitInstructionResponse
from march_shared_msgs.srv import PossibleGaits
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
    POSSIBLE_GAIT_TRANSITIONS = {
        GaitRequest.STAND: ["home_stand", "mpc_stop", "mpc_walk", "mpc_step_and_close"],
        GaitRequest.WALK: ["mpc_stop"],
        GaitRequest.STEP_CLOSE: ["mpc_stop"]
    }

    def __init__(self, node: Node, testing: Bool):
        self._node = node

        self._ping = self._node.get_parameter("ping_safety_node").get_parameter_value().bool_value

        self._instruction_gait_pub = self._node.create_publisher(
            msg_type=GaitInstruction,
            topic="/march/input_device/instruction",
            qos_profile=10,
        )
        self._step_and_hold_step_size_pub = self._node.create_publisher(
            msg_type=String,
            topic="/march/step_and_hold/step_size",
            qos_profile=10,
        )
        self._step_and_hold_start_side_pub = self._node.create_publisher(
            msg_type=String,
            topic="/march/step_and_hold/start_side",
            qos_profile=10,
        )
        self._node.create_subscription(
            msg_type=Bool,
            topic="/march/eeg/on_off",
            qos_profile=1,
            callback=self._update_eeg_on_off,
        )
        self._eeg_on_off_pub = self._node.create_publisher(
            msg_type=Bool,
            topic="/march/eeg/on_off",
            qos_profile=1,
        )
        self._instruction_response_pub = self._node.create_subscription(
            msg_type=GaitInstructionResponse,
            topic="/march/input_device/instruction_response",
            callback=self._response_callback,
            qos_profile=10,
        )
        self._set_gait_control_type = self._node.create_publisher(
            msg_type=String,
            topic="/march/weight_control_type",
            qos_profile=10,
        )
        self.measure_torque_pub = self._node.create_publisher(
            msg_type=Int32,
            topic="/march/measure_torque",
            qos_profile=10,
        )
        self._current_gait = self._node.create_subscription(
            msg_type=String,
            topic="/march/gait/current",
            callback=self._current_gait_callback,
            qos_profile=1,
        )
        self._swing_leg_command_pub = self._node.create_publisher(
            msg_type=Int32,
            topic="/publish_swing_leg_command",
            qos_profile=10,
        )
        self._error_pub = self._node.create_publisher(msg_type=Error, topic="/march/error", qos_profile=10)
        self._possible_gait_client = self._node.create_client(
            srv_type=PossibleGaits, srv_name="/march/gait_selection/get_possible_gaits"
        )
        self.accepted_cb = None
        self.finished_cb = None
        self.rejected_cb = None
        self.current_gait_cb = None
        self.eeg = False
        self._possible_gaits = []

        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        if self._node.get_parameter("use_sim_time").get_parameter_value():
            self._timesource = self._node.create_subscription(
                msg_type=Clock,
                topic="/clock",
                callback=lambda time: None,
                qos_profile=10,
            )

        if self._ping:
            self._alive_pub = self._node.create_publisher(Alive, "/march/input_device/alive", 10)
            self._alive_timer = self._node.create_timer(
                timer_period_sec=0.1,
                callback=self._timer_callback,
                clock=self._node.get_clock(),
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
        self.POSSIBLE_TRANSITIONS = self.POSSIBLE_GAIT_TRANSITIONS

        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        self.gait_future = None
        self.update_possible_gaits()

        # From here is m8 high level control logic, to start with
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
        self.use_mpc = False
        self.eeg = False
        self._current_gait = GaitRequest.FORCE_UNKNOWN


    def __del__(self):
        """Deconstructer, that shutsdown the publishers and resets the timers."""
        self._node.destroy_publisher(self._instruction_gait_pub)
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    def _response_callback(self, msg: GaitInstructionResponse) -> None:
        """Callback for instruction response messages.

        Calls registered callbacks when the gait is accepted, finished or rejected.
        The actual callbacks are defined in InputDeviceView

        Args:
            msg (GaitInstructionResponse): The response this callback reacts to.
        """
        if msg.result == GaitInstructionResponse.GAIT_ACCEPTED and callable(self.accepted_cb):
            self.accepted_cb()
        elif msg.result == GaitInstructionResponse.GAIT_FINISHED and callable(self.finished_cb):
            self.finished_cb()
        elif msg.result == GaitInstructionResponse.GAIT_REJECTED and callable(self.rejected_cb):
            self.rejected_cb()

    def _current_gait_callback(self, msg: String) -> None:
        """Callback for when the current gait changes, sends the msg through to public current_gait_callback.

        Args:
            msg (str): The name of the current gait.
        """
        if callable(self.current_gait_cb):
            self.current_gait_cb(msg.data)

    def _timer_callback(self) -> None:
        """Callback to send out an alive message."""
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)

    def update_possible_gaits(self) -> None:
        """Send out an asynchronous request to get the possible gaits and stores response in gait_future."""
        if self._possible_gait_client.service_is_ready():
            self.gait_future = self._possible_gait_client.call_async(PossibleGaits.Request())
        else:
            while not self._possible_gait_client.wait_for_service(timeout_sec=1):
                self._node.get_logger().warn("Failed to contact possible gaits service")

    def get_possible_gaits(self) -> Future:
        """Returns the future for the names of possible gaits.

        Returns:
            Future. Future for the possible gaits.
        """
        return self.gait_future

    def get_node(self) -> Node:
        """Get function for the node.

        Returns:
             Node. the node that runs the input_device_controller.
        """
        return self._node

    def publish_increment_step_size(self) -> None:
        """Publish a message on `/march/input_device/instruction` to increment the step size."""
        self._node.get_logger().debug("Mock Input Device published step size increment")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.INCREMENT_STEP_SIZE,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_decrement_step_size(self) -> None:
        """Publish a message on `/march/input_device/instruction` to decrement the step size."""
        self._node.get_logger().debug("Mock Input Device published step size decrement")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.DECREMENT_STEP_SIZE,
                gait_name="",
                id=str(self._id),
            )
        )

    @property
    def node(self):
        """Define the node."""
        return self._node

    def measure_torque(self) -> None:
        """Measure the torque values coming in for a few seconds."""
        seconds = 3
        self._node.get_logger().info("Measuring torque for " + str(seconds) + " seconds")
        self.measure_torque_pub.publish(
            Int32(
                data=seconds
            )
        )

    def publish_gait(self, string, control_type) -> None:
        """Publish a message on `/march/input_device/instruction` to publish the gait."""
        self._node.get_logger().debug("Mock Input Device published gait: " + string)
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.GAIT,
                gait_name=string,
                id=str(self._id),
            )
        )
        if control_type is not None:
            self.publish_control_type(control_type)

    def publish_stop(self) -> None:
        """Publish a message on `/march/input_device/instruction` to stop the gait."""
        self._node.get_logger().debug("Mock input device published stop")
        msg = GaitInstruction(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            type=GaitInstruction.STOP,
            gait_name="",
            id=str(self._id),
        )
        self._instruction_gait_pub.publish(msg)

    def publish_continue(self) -> None:
        """Publish a message on `/march/input_device/instruction` to continue the gait."""
        self._node.get_logger().debug("Mock Input Device published continue")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.CONTINUE,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_pause(self) -> None:
        """Publish a message on `/march/input_device/instruction` to pause the gait."""
        self._node.get_logger().debug("Mock Input Device published pause")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.PAUSE,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_error(self) -> None:
        """Publish a fake error message on `/march/error`."""
        self._node.get_logger().debug("Mock Input Device published error")
        self._error_pub.publish(
            Error(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                error_message="Fake error thrown by the develop input device.",
                type=Error.FATAL,
            )
        )

    def switch_to_position(self) -> None:
        """Switches between fuzzy and position control."""
        self._node.get_logger().info("Publishing control type position")
        self._set_gait_control_type.publish(String(data="position"))

    def switch_to_fuzzy(self) -> None:
        """Switches to fuzzy control."""
        self._node.get_logger().info("Publishing control type fuzzy")
        self._set_gait_control_type.publish(String(data="fuzzy"))

    def publish_control_type(self, control_type):
        """Sets the allowed control type depending on the gait."""
        self._node.get_logger().info("Publishing control type " + control_type)
        self._set_gait_control_type.publish(String(data=control_type))

    def publish_sm_to_unknown(self) -> None:
        """Publish a message on `/march/input_device/instruction` that has an unknown instruction."""
        self._node.get_logger().debug("Mock Input Device published state machine to unknown")
        self._instruction_gait_pub.publish(
            GaitInstruction(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                type=GaitInstruction.UNKNOWN,
                gait_name="",
                id=str(self._id),
            )
        )

    def publish_eeg_on_off(self) -> None:
        """Publish eeg on if its off and off if it is on."""
        self._eeg_on_off_pub.publish(Bool(data=not self.eeg))

    def publish_small_narrow(self) -> None:
        """Publish a small_narrow gait on the step_and_hold topic."""
        self._step_and_hold_step_size_pub.publish(String(data="small_narrow"))

    def publish_small_wide(self) -> None:
        """Publish a small_wide gait on the step_and_hold topic."""
        self._step_and_hold_step_size_pub.publish(String(data="small_wide"))

    def publish_large_narrow(self) -> None:
        """Publish a large_narrow gait on the step_and_hold topic."""
        self._step_and_hold_step_size_pub.publish(String(data="large_narrow"))

    def publish_large_wide(self) -> None:
        """Publish a large_wide gait on the step_and_hold topic."""
        self._step_and_hold_step_size_pub.publish(String(data="large_wide"))

    def publish_start_with_left(self) -> None:
        """Publish that a step_and_hold starts from left_swing."""
        self._step_and_hold_start_side_pub.publish(String(data="left_swing"))

    def publish_start_with_right(self) -> None:
        """Publish that a step_and_hold starts from right_swing."""
        self._step_and_hold_start_side_pub.publish(String(data="right_swing"))

    def _update_eeg_on_off(self, msg: Bool) -> None:
        """Update eeg value for when it is changed in the state machine."""
        self.eeg = msg.data

    def _gait_response_callback(self, msg: GaitResponse):
        """Update current node from the state machine."""
        self._current_gait = msg.gait_type

    def publish_mpc_gait(self, gait_type):
        """Publish a message on `/march/gait_request` to publish the gait."""
        self._current_gait = gait_type
        msg = GaitRequest(
            header=Header(stamp=self._node.get_clock().now().to_msg()),
            gait_type=gait_type,
            id=str(self._id),
        )
        self._send_gait_request.publish(msg)
        if gait_type in [2, 3]:
            zero_swing_msg = Int32()
            zero_swing_msg.data = 0
            self._swing_leg_command_pub.publish(zero_swing_msg)

        if gait_type == 5:
            error_msg = Error()
            error_msg.error_message = "Error button clicked on IPD"
            error_msg.type = 0
            self._error_pub.publish(error_msg)

    def update_mpc_gaits(self):
        """Update the possible gait that can be selected by the IPD."""
        return self.POSSIBLE_TRANSITIONS[self._current_gait]

