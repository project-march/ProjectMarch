"""Author: Tuhin Das, MVII."""

import select
import socket
import json
import time
from functools import partial

from rclpy.impl.rcutils_logger import RcutilsLogger as Logger

from march_shared_msgs.msg import CurrentGait, CurrentState
from march_utility.utilities.duration import Duration
from .wireless_ipd_controller import WirelessInputDeviceController
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_gait_selection.dynamic_interpolation.camera_point_handlers.camera_points_handler import (
    FOOT_LOCATION_TIME_OUT,
)
from rclpy.node import Node

HEARTBEAT_TIMEOUT = Duration(0.5)


class ConnectionManager:
    """Class that maintains the wireless IPD connection and communication.

    This manager waits for heartbeat or gait request messages from the wireless IPD. The messages are received
    in the loop of `_wait_for_request()`. After sending a response to the IPD, the node waits for a final
    confirmation message. While waiting for this confirmation, no other messages are received.

    Args:
        host (str): IP address for the connection.
        port (int): Port to connect the sockets on for the wireless connection.
        controller (WirelessInputDeviceController): Controller that publishes gaits.
        node (Node): The node to run the wireless IPD process on.
        logger (Logger): Used to log messages to the terminal with the class name as a prefix.

    Attributes:
        _socket (socket): Socket object used to create a connection.
        _connection (socket): Established socket connection used to send and receive messages.
        _node (Node): The node to run the wireless IPD process on.
        _controller (WirelessInputDeviceController): Controller that publishes gaits.
        _logger (Logger): sed to log messages to the terminal with the class name as a prefix.
        _seq (int): The current sequence number of the messages between the node and the IPD.
        _pause_receiving_messages (bool): Whether the main loop should stop receiving messages for a while.
        _requested_gait (str): The name of a gait that is requested, and None if no gait is requested.
        _current_gait (str): The current gait of the exoskeleton, runnin or just finished.
        _last_heartbeat (Time): Last time a message has been received from the wireless IPD.
        _stopped (bool): Whether stop was pressed on the IPD.
        _last_left_displacement (List): Last displacement found for the left foot with the cameras.
        _last_right_displacement (List): Last displacement found for the right foot with the cameras.
        _last_left_point_timestamp (Time): Last time a left point has been found.
        _last_right_point_timestamp (Time): Last time a right point has been found.

    Subscriptions:
    - /march/foot_position/left
    - /march/foot_position/right
    """

    def __init__(self, host: str, port: int, controller: WirelessInputDeviceController, node: Node, logger: Logger):
        """Constructor."""
        self._connection = None
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((host, port))
        self._socket.listen()
        self._node = node
        self._controller = controller
        self._logger = logger
        self._seq = 0
        self._pause_receiving_messages = False
        self._requested_gait = None
        self._current_gait = "unknown"
        self._last_heartbeat = self._node.get_clock().now()
        self._stopped = False
        self._last_left_point = [0, 0, 0]
        self._last_right_point = [0, 0, 0]
        self._last_left_point_timestamp = self._node.get_clock().now()
        self._last_right_point_timestamp = self._node.get_clock().now()
        self._controller.accepted_cb = partial(self._send_message_till_confirm, "Accepted", True)
        self._controller.rejected_cb = partial(self._send_message_till_confirm, "Reject")
        self._controller.current_gait_cb = self._current_gait_cb
        self._controller.current_state_cb = self._current_state_cb

        self._subscription_left = self._node.create_subscription(
            FootPosition,
            "/march/foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )
        self._subscription_right = self._node.create_subscription(
            FootPosition,
            "/march/foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )

    def _current_gait_cb(self, msg: CurrentGait):
        """Callback when the exoskeleton gait is updated."""
        if self._stopped or "close" in msg.subgait:
            self._current_gait = self._requested_gait = "stop"
            self._stopped = False
        elif "home_stand" not in msg.subgait and self._current_gait != msg.gait:
            self._current_gait = msg.gait

    def _current_state_cb(self, msg: CurrentState):
        """Callback when the exoskeleton state is updated."""
        if msg.state == "unknown" or "home" in msg.state:
            self._current_gait = msg.state

    def _callback_left(self, msg: FootPosition):
        """Callback when a new left foot position is found."""
        self._last_left_point = [msg.displacement.x, msg.displacement.y, msg.displacement.z]
        self._last_left_point_timestamp = self._node.get_clock().now()

    def _callback_right(self, msg: FootPosition):
        """Callback when a new right foot position is found."""
        self._last_right_point = [msg.displacement.x, msg.displacement.y, msg.displacement.z]
        self._last_right_point_timestamp = self._node.get_clock().now()

    def _validate_received_data(self, msg: str):
        """Check if a received message is valid or is empty, meaning the connection is broken.

        Args:
            msg (str): The message to validate.
        """
        if msg == "":
            self._logger.warning("Connection lost with wireless IPD (empty message)")
            raise socket.error
        else:
            self._last_heartbeat = self._node.get_clock().now()

    def _wait_for_request(self):
        """Loop that receives heartbeat and gait request messages from the IPD and handles them."""
        while True:
            try:
                # Do not receive messages while a stop or gait has been requested and
                # the manager waits for a receive confirmation from the IPD.
                counter = 0
                while self._pause_receiving_messages and counter < 15:
                    time.sleep(0.20)
                    counter += 1

                req = self._wait_for_message(5.0)

                req = json.loads(req)
                self._seq = req["seq"]
                msg_type = req["type"]

                # Handle various message types
                if msg_type == "Received":
                    continue

                elif msg_type == "GaitRequest":
                    if req["gait"]["gaitName"] == "stop":
                        self._request_stop()
                    else:
                        self._request_gait(req)

                elif msg_type == "Heartbeat":
                    self._send_message_till_confirm(msg_type="Heartbeat")

                elif msg_type == "Information" and "swing" in req["message"]:
                    self._controller.publish_start_side(req["message"])
                    self._send_message_till_confirm(msg_type="Information", message=req["message"])

            except (json.JSONDecodeError, BlockingIOError):
                continue

            # Reset connection when a message has not been received for 5 seconds
            except socket.timeout as e:
                if (self._node.get_clock().now() - self._last_heartbeat) > HEARTBEAT_TIMEOUT:
                    self._logger.warning("Connection lost with wireless IPD (no heartbeat)")
                    raise e

            # Reset connection if another exception occurs
            except socket.error as e:
                raise e

    def _request_stop(self):
        """Stop the current gait."""
        self._pause_receiving_messages = True
        self._controller.update_possible_gaits()
        self._controller.publish_stop()
        self._stopped = True
        self._current_gait = self._requested_gait = "stop"
        self._send_message_till_confirm(msg_type="Accepted", requested_gait=True)

    def _request_gait(self, req: str):
        """Check whether a gait requested on the wireless IPD is available, and publish it to the state machine if so.

        Args:
            req (str): Gait request message from the IPD.
        """
        self._pause_receiving_messages = True

        # Retrieve whether the requested gait is available as next gait state or not
        self._controller.update_possible_gaits()
        future = self._controller.get_possible_gaits()
        counter = 0
        while not future.done() and counter < 50:
            time.sleep(0.010)
            counter += 1

        self._requested_gait = req["gait"]["gaitName"]

        if future.result() is not None and self._requested_gait in future.result().gaits:
            self._controller.publish_gait(self._requested_gait)
        else:
            self._logger.warning("Failed gait: " + self._requested_gait)
            self._send_message_till_confirm(msg_type="Fail")

    def _wait_for_message(self, timeout: float):
        """Wait until a message is received on the socket connection, until the timeout is reached.

        Args:
            timeout (float): Timeout for the wait duration.

        Returns:
            str: Decoded message from the wireless IPD.
        """
        try:
            self._connection.settimeout(timeout)
            data = self._connection.recv(1024).decode("utf-8")
            self._connection.settimeout(None)
            self._validate_received_data(data)
        except (socket.error, ConnectionResetError, BlockingIOError) as e:
            raise e
        return data

    def _send_message(self, msg: str):
        """Send a message via the socket connection.

        Args:
            msg (str): Message to send.
        """
        try:
            msg = msg + "\r\n"
            self._connection.sendall(msg.encode())
        except (BrokenPipeError, InterruptedError, socket.error):
            raise socket.error

    def _send_message_till_confirm(self, msg_type: str, requested_gait: bool = False, message: str = None):
        """Send a message to the wireless IPD until confirmation is received.

        Args:
            msg_type (str): The type of message that is sent.
            requested_gait (bool): Whether this message is a response to a requested gait or not.
            message (str): Optional message to send.
        """
        if requested_gait:
            send_gait = self._requested_gait
        else:
            send_gait = self._current_gait

        if self._connection is None:
            return

        point_left, point_right = self._update_points()

        msg = {
            "type": msg_type,
            "currentGait": send_gait,
            "message": message,
            "seq": self._seq,
            "point_left": point_left,
            "point_right": point_right,
        }

        while True:
            try:
                # Send a message and wait until a "Received" confirmation message is received.
                self._send_message(json.dumps(msg))
                response = self._wait_for_message(5.0)

                if "Received" in response:
                    response = json.loads(response)
                    seq = response["seq"]
                    if seq == self._seq:
                        self._pause_receiving_messages = False
                        self._empty_socket()
                        return
                    else:
                        self._empty_socket()
                        self._seq = seq
                        continue

            except (json.JSONDecodeError, BlockingIOError):
                continue

            # A timeout is triggered if no "Received" confirmation message is read within 5 seconds.
            except socket.timeout as e:
                if (self._node.get_clock().now() - self._last_heartbeat) > HEARTBEAT_TIMEOUT:
                    self._logger.warning("Connection lost with wireless IPD (no heartbeat)")
                    raise e

            except socket.error as e:
                raise e

    def establish_connection(self):
        """Connect with the wireless IPD via a socket connection.

        After a connection has been established, `_wait_for_request()` is called and receives any messages in
        a loop. If any errors are thrown in `_wait_for_request()`, they are caught here and the connection
        is reset. Afterwards `_wait_for_request()` is called again.
        """
        while True:
            try:
                self._connection, _ = self._socket.accept()
                self._logger.info("Wireless IPD connected")
                self._wait_for_request()
            except (socket.timeout, socket.error) as e:
                self._logger.warning(repr(e))
                self._logger.warning("Reconnecting Wireless IPD")

            self._connection.close()

    def _empty_socket(self):
        """Empty all remaining messages on the socket connection."""
        while True:
            input_ready, _, _ = select.select([self._connection], [], [], 0.0)
            if len(input_ready) == 0:
                break
            for s in input_ready:
                s.recv(1)

    def _update_points(self):
        """Update point found with the cameras before sending them to the IPD."""
        now = self._node.get_clock().now()
        left = self._last_left_point if now - self._last_left_point_timestamp <= FOOT_LOCATION_TIME_OUT else None
        right = self._last_right_point if now - self._last_right_point_timestamp <= FOOT_LOCATION_TIME_OUT else None
        return left, right
