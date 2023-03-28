"""Author: Tuhin Das, MVII."""

import select
import socket
import json
import time
from functools import partial

from rclpy.impl.rcutils_logger import RcutilsLogger as Logger
from march_utility.utilities.duration import Duration
from .wireless_ipd_controller import WirelessInputDeviceController
from march_shared_msgs.msg import GaitRequest
from rclpy.node import Node

HEARTBEAT_TIMEOUT = Duration(seconds=5)


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
        node.get_logger().warn("Wireless ipd node starting up with ip: " + str(host) + " and port: " + str(port))
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
        self._controller.accepted_cb = partial(self.send_message_till_confirm, "Accepted", True)
        self._controller.rejected_cb = partial(self.send_message_till_confirm, "Reject")

    def _validate_received_data(self, msg: str):
        """Check if a received message is valid or is empty, meaning the connection is broken.

        Args:
            msg (str): The message to validate.
        """
        if msg == "":
            self._logger.warning("Connection lost with wireless IPD (empty message)")
            raise socket.error
        else:
            self._logger.warning("Heartbeat received")
            self._last_heartbeat = self._node.get_clock().now()

    def _wait_for_request(self):
        """Loop that receives heartbeat and gait request messages from the IPD and handles them."""
        while True:
            self._logger.info("_wait_for_request called")
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

                if msg_type == "GaitRequest":
                    self._send_gait(req["gait"]["gaitName"])

                elif msg_type == "Heartbeat":
                    self._logger.info("Heartbeat received")
                    self.send_message_till_confirm(msg_type="Heartbeat")

                elif msg_type == "Fail":
                    self._send_gait("error")

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

    def _send_gait(self, req: str):
        """Check whether a gait requested on the wireless IPD is available, and publish it to the state machine if so.

        Args:
            req (str): Gait request message from the IPD.
        """
        self._pause_receiving_messages = True

        self._requested_gait = req
        if self._requested_gait == "stand" or self._requested_gait == "stop":
            self._controller.publish_gait(GaitRequest.STAND)
        elif self._requested_gait == "sit":
            self._controller.publish_gait(GaitRequest.SIT)
        elif self._requested_gait == "walk":
            self._controller.publish_gait(GaitRequest.WALK, "step_close")
        elif self._requested_gait == "error":
            self._controller.publish_gait(GaitRequest.ERROR)

    def _wait_for_message(self, timeout: float):
        """Wait until a message is received on the socket connection, until the timeout is reached.

        Args:
            timeout (float): Timeout for the wait duration.

        Returns:
            str: Decoded message from the wireless IPD.
        """
        try:
            self._logger.warning("waiting for message")
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

    def send_message_till_confirm(self, msg_type: str, requested_gait: str, message: str = None):
        """Send a message to the wireless IPD until confirmation is received.

        Args:
            msg_type (str): The type of message that is sent.
            requested_gait (bool): Whether this message is a response to a requested gait or not.
            message (str): Optional message to send.
        """
        if self._connection is None:
            return

        msg = {
            "type": msg_type,
            "currentGait": requested_gait,
            "message": message,
            "seq": self._seq,
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
