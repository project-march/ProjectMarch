"""Author: Tuhin Das, MVII."""

import select
import socket
import json
import time
from functools import partial
from march_shared_msgs.msg import CurrentGait, CurrentState
from march_utility.utilities.logger import Logger
from march_utility.utilities.duration import Duration
from .input_device_controller import WirelessInputDeviceController
from rclpy.node import Node

HEARTBEAT_TIMEOUT = Duration(0.5)


class ConnectionManager:
    """Class that maintains the wireless IPD connection and all wireless communication."""

    def __init__(self, host: str, port: int, controller: WirelessInputDeviceController, node: Node, logger: Logger):
        """Connection manager constructor.

        Args:
            host (str): ip address for the connection
            port (int): port to connect the sockets on for the wireless connection
            controller (WirelessInputDeviceController): controller that publishes gaits
            node (Node): the node to run the wireless IPD process on
            logger (Logger): used to log messages to the terminal with the class name as a prefix
        """
        server_address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(server_address)
        self._socket.listen()
        self._controller = controller
        self._logger = logger
        self._seq = 0
        self._pause_receiving_messages = False
        self._requested_gait = None
        self._current_gait = "unknown"
        self._node = node
        self._last_heartbeat = self._node.get_clock().now()
        self._stopped = False
        self._controller.accepted_cb = partial(self._send_message_till_confirm, "Accepted", True)
        self._controller.rejected_cb = partial(self._send_message_till_confirm, "Reject")
        self._controller.current_gait_cb = self._current_gait_cb
        self._controller.current_state_cb = self._current_state_cb

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

    def _validate_received_data(self, msg: str):
        """Check if a received message is valid or is empty, meaning the connection is broken.

        Args:
            msg (str): the message to validate
        """
        if msg == "":
            self._logger.warning("Connection lost with wireless IPD (empty message)")
            self._empty_socket()
            raise socket.error
        else:
            self._last_heartbeat = self._node.get_clock().now()

    def _wait_for_request(self):
        """Loop that receives heartbeat and gait request messages from the IPD and handles them."""
        while True:
            try:
                counter = 0
                while self._pause_receiving_messages and counter < 15:
                    time.sleep(0.20)
                    counter += 1

                req = self._wait_for_message(5.0)
                self._empty_socket()

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
                    self._logger.info("Switch side to " + req["message"])

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
            req (str): gait request message from the IPD
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
        if self._requested_gait in future.result().gaits:
            self._controller.publish_gait(self._requested_gait)
        else:
            self._logger.warning("Failed gait: " + self._requested_gait)
            self._send_message_till_confirm(msg_type="Fail")

    def _wait_for_message(self, timeout: float):
        """Wait until a message is received on the socket connection, until the timeout is reached.

        Args:
            timeout (float): timeout for the wait duration
        Returns:
            str: decoded message from the wireless IPD
        """
        try:
            self.connection.settimeout(timeout)
            data = self.connection.recv(1024).decode("utf-8")
            self.connection.settimeout(None)
            self._validate_received_data(data)
        except (socket.error, ConnectionResetError, BlockingIOError) as e:
            raise e
        return data

    def _send_message(self, msg: str):
        """Send a message via the socket connection.

        Args:
            msg (str): message to send
        """
        try:
            msg = msg + "\r\n"
            self.connection.sendall(msg.encode())
        except (BrokenPipeError, InterruptedError, socket.error):
            raise socket.error

    def _send_message_till_confirm(self, msg_type: str, requested_gait: bool = False, message: str = None):
        """Send a message to the wireless IPD until confirmation is received.

        Args:
            msg_type (str): the type of message that is sent
            requested_gait (bool): whether this message is a response to a requested gait or not
            message (str): optional message to send
        """
        if requested_gait:
            send_gait = self._requested_gait
        else:
            send_gait = self._current_gait

        if self.connection is None:
            return

        msg = {"type": msg_type, "currentGait": send_gait, "message": message, "seq": self._seq}

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
                        raise socket.error

                self._empty_socket()
            except (json.JSONDecodeError, BlockingIOError):
                continue

            # A timeout is triggered if no "Received" confirmation message is read within 5 seconds.
            except socket.timeout as e:
                if (self._node.get_clock().now() - self._last_heartbeat) > HEARTBEAT_TIMEOUT:
                    self._logger.warning("Connection lost with wireless IPD (no heartbeat)")
                    self._empty_socket()
                    raise e

            except socket.error as e:
                self._empty_socket()
                raise e

    def establish_connection(self):
        """Connect with the wireless IPD via a socket connection."""
        while True:
            try:
                self.connection, self.addr = self._socket.accept()
                self._logger.info("Wireless IPD connected")
                self._wait_for_request()
            except (socket.timeout, socket.error) as e:
                self._logger.warning(repr(e))
                self._logger.warning("Reconnecting Wireless IPD")
            self.connection.close()

    def _empty_socket(self):
        """Empty all remaining messages on the socket connection."""
        while True:
            input_ready, _, _ = select.select([self._socket], [], [], 0.0)
            if len(input_ready) == 0:
                break
            for s in input_ready:
                s.recv(1)
