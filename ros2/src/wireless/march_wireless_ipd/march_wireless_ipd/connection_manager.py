"""Author: Tuhin Das, MVII."""

import select
import socket
import json
import traceback
import time
from functools import partial
from datetime import datetime


class ConnectionManager:
    """Class that maintains the wireless IPD connection and all wireless communication."""

    def __init__(self, host, port, controller, node):
        server_address = (host, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(server_address)
        self.s.listen()
        self.controller = controller
        self.seq = 0
        self.pause_receiving_messages = False
        self.requested_gait = None
        self.current_gait = "unknown"
        self.last_heartbeat = datetime.now()
        self.node = node
        self.stopped = False
        self.controller.accepted_cb = partial(self.send_message_till_confirm, "Accepted", True)
        self.controller.rejected_cb = partial(self.send_message_till_confirm, "Reject")
        self.controller.current_gait_cb = self._current_gait_cb
        self.controller.current_state_cb = self._current_state_cb

    def _current_gait_cb(self, msg):
        """Callback when the exoskeleton gait is updated."""
        if self.stopped or "close" in msg.subgait:
            self.current_gait = self.requested_gait = "stop"
            self.stopped = False
        elif "home_stand" not in msg.subgait and self.current_gait != msg.gait:
            self.current_gait = msg.gait

    def _current_state_cb(self, msg):
        """Callback when the exoskeleton state is updated."""
        if msg.state == "unknown" or msg.state == "home_stand":
            self.current_gait = msg.state

    def validate_received_data(self, msg):
        """Check if a received message is valid or is empty, meaning the connection is broken.

        Args:
            msg (str): the message to validate
        """
        if msg == "":
            self.ros_warning("Connection lost with wireless IPD (empty message)")
            self.empty_socket()
            raise socket.error
        else:
            self.last_heartbeat = datetime.now()

    def wait_for_request(self):
        """Loop that receives heartbeat and gait request messages from the IPD and handles them."""
        while True:
            try:
                counter = 0
                while self.pause_receiving_messages and counter < 15:
                    time.sleep(0.20)
                    counter += 1

                req = self.wait_for_message(5.0)
                self.empty_socket()

                # Handle various message types
                if "Received" in req:
                    continue

                elif "GaitRequest" in req:
                    req = json.loads(req)
                    self.seq = req["seq"]
                    if req["gait"]["gaitName"] == "stop":
                        self.request_stop()
                    else:
                        self.request_gait(req)

                elif "Heartbeat" in req:
                    req = json.loads(req)
                    self.seq = req["seq"]
                    self.send_message_till_confirm(msg_type="Heartbeat")

            except (json.JSONDecodeError, BlockingIOError):
                continue

            # Reset connection when a message has not been received for 5 seconds
            except socket.timeout as e:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 5.0:
                    self.ros_warning("Connection lost with wireless IPD (no heartbeat)")
                    raise e

            # Reset connection if another exception occurs
            except socket.error as e:
                raise e

    def request_stop(self):
        """Stop the current gait."""
        self.pause_receiving_messages = True
        self.controller.update_possible_gaits()
        self.controller.publish_stop()
        self.stopped = True
        self.current_gait = self.requested_gait = "stop"
        self.send_message_till_confirm(msg_type="Accepted", requested_gait=True)

    def request_gait(self, req):
        """Check whether a gait requested on the wireless IPD is available, and publish it to the state machine if so.

        Args:
            req (str): gait request message from the IPD
        """
        self.pause_receiving_messages = True

        # Retrieve whether the requested gait is available as next gait state or not
        self.controller.update_possible_gaits()
        future = self.controller.get_possible_gaits()
        counter = 0
        while not future.done() and counter < 50:
            time.sleep(0.010)
            counter += 1

        self.requested_gait = req["gait"]["gaitName"]
        if self.requested_gait in future.result().gaits:
            self.controller.publish_gait(self.requested_gait)
        else:
            self.ros_warning("Failed gait: " + self.requested_gait)
            self.send_message_till_confirm(msg_type="Fail")

    def wait_for_message(self, timeout):
        """Wait until a message is received on the socket connection, until the timeout is reached.

        Args:
            timeout (double): timeout for the wait duration
        Returns:
            str: decoded message from the wireless IPD
        """
        try:
            self.connection.settimeout(timeout)
            data = self.connection.recv(1024).decode("utf-8")
            self.connection.settimeout(None)
            self.validate_received_data(data)
        except (socket.error, ConnectionResetError, BlockingIOError) as e:
            raise e
        return data

    def send_message(self, msg):
        """Send a message via the socket connection.

        Args:
            msg (str): message to send
        """
        try:
            msg = msg + "\r\n"
            self.connection.sendall(msg.encode())
        except (BrokenPipeError, InterruptedError, socket.error) as e:
            print(e)
            raise socket.error

    def send_message_till_confirm(self, msg_type, requested_gait=False):
        """Send a message to the wireless IPD until confirmation is received.

        Args:
            msg_type (str): the type of message that is sent
            requested_gait (bool): whether this message is a response to a requested gait or not
        """
        if requested_gait:
            send_gait = self.requested_gait
        else:
            send_gait = self.current_gait

        if self.connection is None:
            return

        msg = {"type": msg_type, "currentGait": send_gait, "seq": self.seq}

        while True:
            try:
                # Send a message and wait until a "Received" confirmation message is received.
                self.send_message(json.dumps(msg))
                response = self.wait_for_message(5.0)

                if "Received" in response:
                    response = json.loads(response)
                    seq = response["seq"]
                    if seq == self.seq:
                        self.pause_receiving_messages = False
                        self.empty_socket()
                        return
                    else:
                        raise socket.error

                self.empty_socket()
            except (json.JSONDecodeError, BlockingIOError):
                continue

            # A timeout is triggered if no "Received" confirmation message is read within 5 seconds.
            except socket.timeout as e:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 5.0:
                    self.ros_warning("Connection lost with wireless IPD (no heartbeat)")
                    self.empty_socket()
                    raise e

            except socket.error as e:
                self.empty_socket()
                raise e

    def establish_connection(self):
        """Connect with the wireless IPD via a socket connection."""
        while True:
            try:
                self.connection, self.addr = self.s.accept()
                self.ros_info("Wireless IPD connected")
                self.wait_for_request()
            except (socket.timeout, socket.error):
                self.ros_warning("Reconnecting Wireless IPD")
                # self.ros_warning(traceback.format_exc())
            self.connection.close()

    def empty_socket(self):
        """Empty all remaining messages on the socket connection."""
        while True:
            input_ready, _, _ = select.select([self.s], [], [], 0.0)
            if len(input_ready) == 0:
                break
            for s in input_ready:
                s.recv(1)

    def ros_info(self, msg):
        """Print ros info message."""
        self.controller.get_node().get_logger().info(msg)

    def ros_warning(self, msg):
        """Print ros warning message."""
        self.controller.get_node().get_logger().warning(msg)
